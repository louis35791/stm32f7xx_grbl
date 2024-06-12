#include <string.h>
#include <limits.h>
#include "stm32f7xx_hal.h"
#include "stm32f7xx_grbl.h"
#include "stm32f7xx_timer_extension.h"
#include "FreeRTOS.h"
#include "task.h"
#include "step.h"

#define PULSE_FREQ 100000  // Hz
#define HALF_PERIOD 0x7FFF // 0xFFFF / 2
#define RING_BUFFER_SIZE 6
#define GAP_HEAD_TAIL 1 // gap between head and tail, two buffers in used by DMA
#define STEP_INCREMENT 5

/**
 * Type Defines
 */
typedef struct TIM_DMA_Parameters
{
    TIM_HandleTypeDef *htim;
    uint8_t TIM_DMA_ID;
    uint8_t TIM_DMA_ID_CC;
    uint8_t TIM_CHANNEL;
    uint32_t CCRx_Addr;
    uint32_t CompareEventID;
    uint32_t lastCounterValue;
} TIM_DMA_Parameters_t;

typedef struct
{
    doubleBufferArray_t data;
    uint16_t length;
} pulse_t;

typedef struct
{
    uint32_t counter;
    uint32_t steps;
    uint16_t buf_index;
} stepper_t;

/**
 * Declare Variables
 */
// Ring buffer for each axis
DEFINE_RING_BUFFER(x, RING_BUFFER_SIZE);
DEFINE_RING_BUFFER(y, RING_BUFFER_SIZE);
DEFINE_RING_BUFFER(z, RING_BUFFER_SIZE);

// ring buffer head and tail for each axis
volatile uint16_t ringBufferHead[NUM_DIMENSIONS] = {0};
volatile uint16_t ringBufferTail[NUM_DIMENSIONS] = {0};

// declare an array of timer and DMA parameters for each axis
static TIM_DMA_Parameters_t axisTimerDMAParams[NUM_DIMENSIONS];

// declare an array of stepper_t for each axis
static stepper_t stepper[NUM_DIMENSIONS];

// declare an array of general notification
volatile uint32_t generalNotification;

// increment for pulse
volatile uint16_t increment = 0;

// counter overruns the batch update threshold
volatile uint8_t overrun = 0;

// record the axes that their DMA transfer is completed
volatile uint8_t completedDMATransferAxes = 0; // bit 0: x axis, bit 1: y axis, bit 2: z axis

// current counter value only for calculating the pulse data
volatile uint32_t currentCounterValue = MINIMUN_LOW_PULSE_WIDTH_TICKS;

/**
 * Function Prototypes
 */
static void TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma);
void stepCalculatePulseData();
void stepUpdateBuffer(doubleBufferArray_t *buffer, uint32_t start_value, uint32_t increment, size_t length);
HAL_StatusTypeDef stepTimeOCStartDMA(TIM_HandleTypeDef *htim, uint32_t Channel, const uint32_t *pData, uint16_t Length);
uint16_t stepRingBufferGetNext(axis_t axis);
void stepRingBufferIncrementHead(axis_t axis);
uint16_t stepRingBufferGetTail(axis_t axis);
void stepRingBufferIncrementTail(axis_t axis);
uint32_t stepGetFreeDataAddress(axis_t axis);
uint32_t stepGetAvailableDataAddress(axis_t axis);

/* ============================= */
/* === Function Declarations === */
/* ============================= */
// initialization of step function
void stepInit(void)
{
    // initialize axis parameters
    INIT_TIM_DMA_PARAMETERS(axisTimerDMAParams, X_AXIS);
    INIT_TIM_DMA_PARAMETERS(axisTimerDMAParams, Y_AXIS);
    INIT_TIM_DMA_PARAMETERS(axisTimerDMAParams, Z_AXIS);

    // initialize stepper parameters
    INIT_STEPPER_ARRAY(stepper, X_AXIS);
    INIT_STEPPER_ARRAY(stepper, Y_AXIS);
    INIT_STEPPER_ARRAY(stepper, Z_AXIS);

    // ========================================
    // starting speed of each axis
    // ========================================
    stepper[X_AXIS].steps = 1;
    stepper[Y_AXIS].steps = 1;
    stepper[Z_AXIS].steps = 1;
}

// handler for step task
void stepTask(void *pvParameters)
{
    // initialize step function
    stepInit();

    // set PD3 to high to enable level shifter
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);

    // set general notification to data not available to start the process
    generalNotification |= (GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE | GENERAL_NOTIFICATION_FIRST_TIME_START);

    // Infinite loop
    for (;;)
    {
        // prepare pulse data
        stepCalculatePulseData();

        // check if there is a notification to resume DMA stream asking from the timer interrupt
        if (!(generalNotification & GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE))
        {
            continue;
        }

        // check if there is any buffer available
        if (stepRingBufferGetTail(Z_AXIS) == UINT16_MAX)
        {
            continue;
        }

        // clear the flag
        generalNotification &= ~GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE;

        // check if it is the first time to start the process
        if (generalNotification & GENERAL_NOTIFICATION_FIRST_TIME_START)
        {
            // clear the flag
            generalNotification &= ~GENERAL_NOTIFICATION_FIRST_TIME_START;

            // set up each axis
            for (int8_t i = NUM_DIMENSIONS - 1; i >= 0; i--)
            {
                // get timer and DMA parameters of this axis
                TIM_DMA_Parameters_t *timDMAParamsPulse = &axisTimerDMAParams[i];

                // reset timer
                __HAL_TIM_SetCounter(timDMAParamsPulse->htim, 0);

                // get available data address
                uint32_t address = stepGetAvailableDataAddress(i);

                // start timer output mode with DMA stream
                stepTimeOCStartDMA(timDMAParamsPulse->htim, timDMAParamsPulse->TIM_CHANNEL, (uint32_t *)address, ((pulse_t *)address)->length);

                // generate compare event
                HAL_TIM_GenerateEvent(timDMAParamsPulse->htim, timDMAParamsPulse->CompareEventID);

                // check if it is the master timer
                if (timDMAParamsPulse->htim != &MASTER_TIM_HANDLE)
                {
                    // enable the timer
                    __HAL_TIM_ENABLE(timDMAParamsPulse->htim);
                }
            }

            // enable the master timer
            __HAL_TIM_ENABLE(&MASTER_TIM_HANDLE);
        }
        else // not the first time to start the process, but out of data asking from the timer interrupt
        {
            // set up each axis
            for (uint8_t i = 0; i < NUM_DIMENSIONS; i++)
            {
                // get timer and DMA parameters of this axis
                TIM_DMA_Parameters_t *timDMAParamsPulse = &axisTimerDMAParams[i];

                // get available data address
                uint32_t address = stepGetAvailableDataAddress(i);

                // resume DMA stream with updated buffer and length
                RESUME_DMA_STREAM_WITH_TC(timDMAParamsPulse->htim->hdma[timDMAParamsPulse->TIM_DMA_ID], address, ((pulse_t *)address)->length);
            }

            // restart counter
            TIM_START_COUNTER(MASTER_TIM_HANDLE);
        }
    }
}

void stepCalculatePulseData()
{
    static uint8_t getNewBuffer = (1 << NUM_DIMENSIONS) - 1; // bit 0: x axis, bit 1: y axis, bit 2: z axis
    static pulse_t *pulse[NUM_DIMENSIONS] = {0};
    static uint32_t oneSecondCounter = 0; // counter for one second, i.e. up to APB1 clock frequency
    static uint8_t rampAccel = 1;         // flag to indicate the acceleration phase

    // loop through all axes
    for (uint8_t i = 0; i < NUM_DIMENSIONS; i++)
    {
        stepper_t *st = &stepper[i];

        // determine if a new buffer is needed
        if (getNewBuffer & (1 << i))
        {
            // get new buffer
            pulse[i] = (pulse_t *)stepGetFreeDataAddress(i);

            // check if the buffer is available
            if (pulse[i] == 0)
            {
                return;
            }

            // reset buffer length
            pulse[i]->length = 0;

            // reset the buffer index
            st->buf_index = 0;

            // initialize stepper parameters
            st->counter = STEP_EVENT_COUNT >> 1;
            st->buf_index = 0;

            // reset the flag
            getNewBuffer &= ~(1 << i);
        }

        // determine if a pair of pulse data should be added in this axis or not by Bresenham line algorithm.
        // a pair of pulse data consists of a pulse data for the on state and a pulse data for the off state.
        st->counter += st->steps;

        if (st->counter > STEP_EVENT_COUNT)
        {
            st->counter -= STEP_EVENT_COUNT;

            // add pulse data
            // add ON state pulse data
            pulse[i]->data[st->buf_index] = currentCounterValue;
            st->buf_index++;
            // add OFF state pulse data
            pulse[i]->data[st->buf_index] = currentCounterValue + PULSE_WIDTH_TICKS;
            st->buf_index++;

            // increment the length of available data
            pulse[i]->length += 2;

            // check if the buffer is full
            if (st->buf_index >= DOUBLE_BUFFER_SIZE)
            {
                // set the flag to get new buffer
                getNewBuffer |= (1 << i);
            }
        }
    }

    // update variables
    currentCounterValue += MINIMUM_PULSE_PERIOD_TICKS;

    // check if any buffer is full of data
    if (getNewBuffer > 0)
    {
        // increment ring buffer head for all axes
        stepRingBufferIncrementHead(X_AXIS);
        stepRingBufferIncrementHead(Y_AXIS);
        stepRingBufferIncrementHead(Z_AXIS);

        getNewBuffer = (1 << NUM_DIMENSIONS) - 1;
    }

    if (rampAccel == 0)
        return;

    oneSecondCounter += MINIMUM_PULSE_PERIOD_TICKS;

    if (oneSecondCounter >= APB1_TIMER_CLOCK)
    {
        oneSecondCounter -= APB1_TIMER_CLOCK;

        stepper[X_AXIS].steps += STEP_INCREMENT;
        if (stepper[X_AXIS].steps > STEP_X)
        {
            stepper[X_AXIS].steps = STEP_X;
            rampAccel = 0;
        }

        stepper[Y_AXIS].steps += STEP_INCREMENT;
        if (stepper[Y_AXIS].steps > STEP_Y)
        {
            stepper[Y_AXIS].steps = STEP_Y;
        }

        stepper[Z_AXIS].steps += STEP_INCREMENT;
        if (stepper[Z_AXIS].steps > STEP_Z)
        {
            stepper[Z_AXIS].steps = STEP_Z;
        }
    }
}

/**
 * @brief Update double buffer with increment value
 */
void stepUpdateBuffer(doubleBufferArray_t *buffer, uint32_t start_value, uint32_t increment, size_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        (*buffer)[i] = (uint32_t)(start_value + increment * (i / 2));
        i++;
        (*buffer)[i] = (uint32_t)(start_value + increment * (i / 2) + PULSE_WIDTH_TICKS);
    }
}

/**
 * @brief  Starts the TIM Output Compare signal generation in DMA mode.
 * @param  htim TIM Output Compare handle
 * @param  Channel TIM Channel to be enabled
 *          This parameter can be one of the following values:
 *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
 *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
 *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
 *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
 * @param  pData The source Buffer address.
 * @param  Length The length of data to be transferred from memory to TIM peripheral
 * @retval HAL status
 */
HAL_StatusTypeDef stepTimeOCStartDMA(TIM_HandleTypeDef *htim, uint32_t Channel, const uint32_t *pData,
                                     uint16_t Length)
{
    HAL_StatusTypeDef status = HAL_OK;
    //   uint32_t tmpsmcr;

    /* Check the parameters */
    assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, Channel));

    /* Set the TIM channel state */
    if (TIM_CHANNEL_STATE_GET(htim, Channel) == HAL_TIM_CHANNEL_STATE_BUSY)
    {
        return HAL_BUSY;
    }
    else if (TIM_CHANNEL_STATE_GET(htim, Channel) == HAL_TIM_CHANNEL_STATE_READY)
    {
        if ((pData == NULL) || (Length == 0U))
        {
            return HAL_ERROR;
        }
        else
        {
            TIM_CHANNEL_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_BUSY);
        }
    }
    else
    {
        return HAL_ERROR;
    }

    switch (Channel)
    {
    case TIM_CHANNEL_1:
    {
        /* Set the DMA compare callbacks */
        htim->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = TIM_DMADelayPulseCplt;
        htim->hdma[TIM_DMA_ID_CC1]->XferHalfCpltCallback = TIM_DMADelayPulseHalfCplt;

        /* Set the DMA error callback */
        htim->hdma[TIM_DMA_ID_CC1]->XferErrorCallback = TIM_DMAError;

        /* Enable the DMA stream */
        if (HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC1], (uint32_t)pData, (uint32_t)&htim->Instance->CCR1,
                             Length) != HAL_OK)
        {
            /* Return error status */
            return HAL_ERROR;
        }

        /* Enable the TIM Capture/Compare 1 DMA request */
        __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC1);
        break;
    }

    case TIM_CHANNEL_2:
    {
        /* Set the DMA compare callbacks */
        htim->hdma[TIM_DMA_ID_CC2]->XferCpltCallback = TIM_DMADelayPulseCplt;
        htim->hdma[TIM_DMA_ID_CC2]->XferHalfCpltCallback = TIM_DMADelayPulseHalfCplt;

        /* Set the DMA error callback */
        htim->hdma[TIM_DMA_ID_CC2]->XferErrorCallback = TIM_DMAError;

        /* Enable the DMA stream */
        if (HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC2], (uint32_t)pData, (uint32_t)&htim->Instance->CCR2,
                             Length) != HAL_OK)
        {
            /* Return error status */
            return HAL_ERROR;
        }

        /* Enable the TIM Capture/Compare 2 DMA request */
        __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC2);
        break;
    }

    case TIM_CHANNEL_3:
    {
        /* Set the DMA compare callbacks */
        htim->hdma[TIM_DMA_ID_CC3]->XferCpltCallback = TIM_DMADelayPulseCplt;
        htim->hdma[TIM_DMA_ID_CC3]->XferHalfCpltCallback = TIM_DMADelayPulseHalfCplt;

        /* Set the DMA error callback */
        htim->hdma[TIM_DMA_ID_CC3]->XferErrorCallback = TIM_DMAError;

        /* Enable the DMA stream */
        if (HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC3], (uint32_t)pData, (uint32_t)&htim->Instance->CCR3,
                             Length) != HAL_OK)
        {
            /* Return error status */
            return HAL_ERROR;
        }
        /* Enable the TIM Capture/Compare 3 DMA request */
        __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC3);
        break;
    }

    case TIM_CHANNEL_4:
    {
        /* Set the DMA compare callbacks */
        htim->hdma[TIM_DMA_ID_CC4]->XferCpltCallback = TIM_DMADelayPulseCplt;
        htim->hdma[TIM_DMA_ID_CC4]->XferHalfCpltCallback = TIM_DMADelayPulseHalfCplt;

        /* Set the DMA error callback */
        htim->hdma[TIM_DMA_ID_CC4]->XferErrorCallback = TIM_DMAError;

        /* Enable the DMA stream */
        if (HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC4], (uint32_t)pData, (uint32_t)&htim->Instance->CCR4,
                             Length) != HAL_OK)
        {
            /* Return error status */
            return HAL_ERROR;
        }
        /* Enable the TIM Capture/Compare 4 DMA request */
        __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC4);
        break;
    }

    default:
        status = HAL_ERROR;
        break;
    }

    if (status == HAL_OK)
    {
        /* Enable the Output compare channel */
        TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_ENABLE);

        if (IS_TIM_BREAK_INSTANCE(htim->Instance) != RESET)
        {
            /* Enable the main output */
            __HAL_TIM_MOE_ENABLE(htim);
        }
    }

    /* Return function status */
    return status;
}

/**
 * @brief  TIM DMA Delay Pulse complete callback.
 * @param  hdma pointer to DMA handle.
 * @retval None
 */
static void TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma)
{
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    if (hdma == htim->hdma[TIM_DMA_ID_CC1])
    {
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_1;

        if (hdma->Init.Mode == DMA_NORMAL)
        {
            TIM_CHANNEL_STATE_SET(htim, TIM_CHANNEL_1, HAL_TIM_CHANNEL_STATE_READY);
        }
    }
    else if (hdma == htim->hdma[TIM_DMA_ID_CC2])
    {
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_2;

        if (hdma->Init.Mode == DMA_NORMAL)
        {
            TIM_CHANNEL_STATE_SET(htim, TIM_CHANNEL_2, HAL_TIM_CHANNEL_STATE_READY);
        }
    }
    else if (hdma == htim->hdma[TIM_DMA_ID_CC3])
    {
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_3;

        if (hdma->Init.Mode == DMA_NORMAL)
        {
            TIM_CHANNEL_STATE_SET(htim, TIM_CHANNEL_3, HAL_TIM_CHANNEL_STATE_READY);
        }
    }
    else if (hdma == htim->hdma[TIM_DMA_ID_CC4])
    {
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_4;

        if (hdma->Init.Mode == DMA_NORMAL)
        {
            TIM_CHANNEL_STATE_SET(htim, TIM_CHANNEL_4, HAL_TIM_CHANNEL_STATE_READY);
        }
    }
    else
    {
        /* nothing to do */
    }

#if (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->PWM_PulseFinishedCallback(htim);
#else
    HAL_TIM_PWM_PulseFinishedCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */

    htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
}

/**
 * @brief  TIM DMA Pulse complete callback.
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    // stop counter
    TIM_STOP_COUNTER(MASTER_TIM_HANDLE); // timer x axis

    // set PD6 to high ===> signal the start of ISR
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);

    // identify the axis
    axis_t axis = GET_AXIS_FROM_TIM_HANDLE(htim);

    /**
     * Here we disable the following checking for a better performance.
     */
    // if (axis == NUM_DIMENSIONS)
    // {
    //     // resume counter
    //     TIM_START_COUNTER(MASTER_TIM_HANDLE); // timer x axis

    //     // set PD6 to low ===> signal the end of ISR
    //     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);

    //     return;
    // }

    // set the axis that its DMA transfer is completed
    completedDMATransferAxes |= (1 << axis);

    // check if all axes have completed their DMA transfer
    if (completedDMATransferAxes == (1 << NUM_DIMENSIONS) - 1)
    {
        // reset the axes that their DMA transfer is completed
        completedDMATransferAxes = 0;

        // set generatl notification to all axes have completed their DMA transfer
        generalNotification |= GENERAL_NOTIFICATION_ALL_AXES_DMA_COMPLETED;
    }

    // suspend DMA stream
    // ==> to update DMA buffer length and address, DMA stream should be suspended
    SUSPEND_DMA_STREAM(htim->hdma[GET_TIM_DMA_ID_FROM_AXIS(axis)]);

    // get the available data address
    uint32_t address = stepGetAvailableDataAddress(axis);

    // check if the address is valid
    if (address != 0)
    {
        // set PD7 to high ===> signal the start of updating DMA buffer
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

        // resume DMA stream with updated buffer and length
        RESUME_DMA_STREAM_WITH_TC(htim->hdma[GET_TIM_DMA_ID_FROM_AXIS(axis)], address, ((pulse_t *)address)->length);

        // restart counter
        TIM_START_COUNTER(MASTER_TIM_HANDLE);

        // set PD7 to low ===> signal the end of updating DMA buffer
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
    }
    else
    {
        // notify main task to update pulse data
        generalNotification |= GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE;
    }

    // set PD6 to low ===> signal the end of ISR
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
}

/**
 * Double Bufffer Mode Callback - M0 Transfer Complete
 */
void PingPongM0TransferCompleteCallback(TIM_HandleTypeDef *htim)
{
    // set PD6 to high ===> signal the start of ISR
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);

    axis_t axis = GET_AXIS_FROM_TIM_HANDLE(htim);

    if (axis == NUM_DIMENSIONS)
        return;

    uint32_t address = stepGetAvailableDataAddress(axis);

    if (address == 0) // no data available
    {
        // stop counter
        TIM_STOP_COUNTER(MASTER_TIM_HANDLE); // timer x axis

        // set notification to data not available
        generalNotification |= GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE;

        return;
    }

    if (axis == Y_AXIS)
    {
        // debug here
    }

    // update M0 buffer address
    htim->hdma[GET_TIM_DMA_ID_FROM_AXIS(axis)]->Instance->M0AR = address;

    // set PD6 to low ===> signal the end of ISR
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
}

/**
 * Double Bufffer Mode Callback - M1 Transfer Complete
 */
void PingPongM1TransferCompleteCallback(TIM_HandleTypeDef *htim)
{
    // set PD7 to high ===> signal the start of ISR
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

    axis_t axis = GET_AXIS_FROM_TIM_HANDLE(htim);

    if (axis == NUM_DIMENSIONS)
        return;

    uint32_t address = stepGetAvailableDataAddress(axis);

    if (address == 0) // no data available
    {
        // stop counter
        TIM_STOP_COUNTER(MASTER_TIM_HANDLE); // timer x axis

        // set notification to data not available
        generalNotification |= GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE;

        return;
    }

    if (axis == Y_AXIS)
    {
        // debug here
    }

    // update M1 buffer address
    htim->hdma[GET_TIM_DMA_ID_FROM_AXIS(axis)]->Instance->M1AR = address;

    // set PD7 to low ===> signal the end of ISR
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
}

/**
 * Ring Buffer Manipulation Functions
 */
uint16_t stepRingBufferGetNext(axis_t axis)
{
    /**
     * In order to avoid the data updating overwriting the data used by DMA,
     * the head and tail pointers are separated by a gap of N buffers.
     * N is defined by GAP_HEAD_TAIL. Howerver, instead of tracking the gap
     * to determine if there is a next free index, here we simply check if
     * the length of the populated data of ring buffer is less than
     * RING_BUFFER_SIZE - GAP_HEAD_TAIL.
     *
     * i.e.
     * | * | * | * | (Head)>> | 0 | 0 | >>(Tail) | * | * | * |
     * >-------- Body ------->|  GAP  |>------- Body -------->
     */
    uint16_t bodyLength = (ringBufferHead[axis] - ringBufferTail[axis] + RING_BUFFER_SIZE) % RING_BUFFER_SIZE;

    if (bodyLength < RING_BUFFER_SIZE - GAP_HEAD_TAIL)
    {
        return (ringBufferHead[axis] + 1) % RING_BUFFER_SIZE;
    }
    else
        return UINT16_MAX;
}

void stepRingBufferIncrementHead(axis_t axis)
{
    uint16_t nextIndex = stepRingBufferGetNext(axis);

    if (nextIndex != UINT16_MAX)
    {
        ringBufferHead[axis] = nextIndex;
    }
}

uint16_t stepRingBufferGetTail(axis_t axis)
{
    if (ringBufferTail[axis] != ringBufferHead[axis])
    {
        return ringBufferTail[axis];
    }
    else
        return UINT16_MAX;
}

void stepRingBufferIncrementTail(axis_t axis)
{
    RING_BUFFER_INCREMENT_TAIL(axis, RING_BUFFER_SIZE);
}

/**
 * Get the free data address
 * @param axis Axis to get the free data address
 * @return Free data address; 0 if no data available
 */
uint32_t stepGetFreeDataAddress(axis_t axis)
{
    if (stepRingBufferGetNext(axis) == UINT16_MAX)
        return 0;

    uint16_t head = ringBufferHead[axis];

    switch (axis)
    {
    case X_AXIS:
        return (uint32_t)(&xPulseBuffer[head]);
    case Y_AXIS:
        return (uint32_t)(&yPulseBuffer[head]);
    case Z_AXIS:
        return (uint32_t)(&zPulseBuffer[head]);
    case NUM_DIMENSIONS:
    default:
        return 0;
    }
}

/**
 * Get the available data address
 * @param axis Axis to get the available data address
 * @return Available data address; 0 if no data available
 */
uint32_t stepGetAvailableDataAddress(axis_t axis)
{
    uint16_t tail = stepRingBufferGetTail(axis);

    if (tail == UINT16_MAX)
        return 0;

    stepRingBufferIncrementTail(axis);

    switch (axis)
    {
    case X_AXIS:
        return (uint32_t)(&xPulseBuffer[tail]);
    case Y_AXIS:
        return (uint32_t)(&yPulseBuffer[tail]);
    case Z_AXIS:
        return (uint32_t)(&zPulseBuffer[tail]);
    case NUM_DIMENSIONS:
    default:
        return 0;
    }
}
