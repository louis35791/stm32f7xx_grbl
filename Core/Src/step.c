#include <string.h>
#include <limits.h>
#include "stm32f7xx_hal.h"
#include "stm32f7xx_grbl.h"
#include "stm32f7xx_timer_extension.h"
#include "FreeRTOS.h"
#include "task.h"
#include "step.h"

#define PULSE_FREQ 100000   // Hz
#define HALF_PERIOD 0x7FFF // 0xFFFF / 2
#define RING_BUFFER_SIZE 6
#define GAP_HEAD_TAIL 2 // gap between head and tail, two buffers in used by DMA

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
    uint32_t lastCounterValue;
    uint16_t lengthAvailableData;
} TIM_DMA_Parameters_t;

typedef struct
{
    TIM_DMA_Parameters_t pulse;
    TIM_DMA_Parameters_t dir;
} Axis_TIM_DMA_Parameters_t;

typedef struct
{
    doubleBufferArray_t pulse[2];
    doubleBufferArray_t dir[2];
} AxisBuffer_t;

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

// declare an array of buffer for each axis
// static AxisBuffer_t axisBuffer[NUM_DIMENSIONS];

// declare an array of general notification
volatile uint32_t generalNotification;

// threshold for determining whether to batch update pulse data.
volatile uint32_t updateBatchThreshold = 0;
volatile uint32_t nextUpdateBatchThreshold = 0;

// increment for pulse
volatile uint16_t increment = 0;

// counter overruns the batch update threshold
volatile uint8_t overrun = 0;

// capture compare register value
volatile uint16_t captureCompareReg = 0;

/**
 * Function Prototypes
 */
static void TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma);
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
}

// handler for step task
void stepTask(void *pvParameters)
{
    // initialize step function
    stepInit();

    // set PD3 to high to enable level shifter
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);

    // compute increment for pulse
    // increment = COMPUTE_INCREMENT_FOR_PULSE(1000, APB1_TIMER_CLOCK);
    increment = COMPUTE_INCREMENT_FOR_PULSE(PULSE_FREQ, APB1_TIMER_CLOCK);
    uint32_t offStatePulseWidth = increment - PULSE_WIDTH_TICKS;

    /**
     * Prepare buffer for each axis in a reverse order
     * to ensure that all the slave timers are ready
     * before the master timer starts.
     */
    for (int8_t i = NUM_DIMENSIONS - 1; i >= 0; i--)
    {
        // get timer and DMA parameters of this axis
        TIM_DMA_Parameters_t *timDMAParamsPulse = &axisTimerDMAParams[i];

        timDMAParamsPulse->lastCounterValue = 10;

        // update all buffer data in ring buffer except the last one
        for (uint16_t j = 0; j < RING_BUFFER_SIZE - GAP_HEAD_TAIL; j++)
        {
            // put data into ring buffer head
            doubleBufferArray_t *pulseBuffer = (doubleBufferArray_t *)stepGetFreeDataAddress(i);

            if (pulseBuffer == 0)
                Error_Handler();

            // update pulse data
            stepUpdateBuffer(pulseBuffer, timDMAParamsPulse->lastCounterValue + offStatePulseWidth, increment, DOUBLE_BUFFER_SIZE);

            // increment ring buffer head
            stepRingBufferIncrementHead(i);

            // set last counter value
            timDMAParamsPulse->lastCounterValue = (*pulseBuffer)[DOUBLE_BUFFER_SIZE - 1];

            // set update batch threshold
            if (i == 0 && j == 0)
                updateBatchThreshold = timDMAParamsPulse->lastCounterValue;
        }

        // request two buffers ready for transfer
        doubleBufferArray_t *pulseBufferM0 = (doubleBufferArray_t *)stepGetAvailableDataAddress(i);
        doubleBufferArray_t *pulseBufferM1 = (doubleBufferArray_t *)stepGetAvailableDataAddress(i);

        // reset timer counter
        __HAL_TIM_SET_COUNTER(timDMAParamsPulse->htim, 0);

        // set compare value to the first element of the buffer in timer
        // __HAL_TIM_SET_COMPARE(timDMAParamsPulse->htim, timDMAParamsPulse->TIM_CHANNEL, 10);

        // start double buffer mode
        TIM_OC_Start_DMA_Double_Buffer(timDMAParamsPulse->htim, timDMAParamsPulse->TIM_CHANNEL, (uint32_t)pulseBufferM0, timDMAParamsPulse->CCRx_Addr, (uint32_t)pulseBufferM1, DOUBLE_BUFFER_SIZE);

        // generate compare event
        switch (i)
        {
        case X_AXIS:
            HAL_TIM_GenerateEvent(timDMAParamsPulse->htim, TIM_EVENTSOURCE_CC3);
            __HAL_TIM_ENABLE(timDMAParamsPulse->htim);
            break;
        case Y_AXIS:
            HAL_TIM_GenerateEvent(timDMAParamsPulse->htim, TIM_EVENTSOURCE_CC4);
            break;
        case Z_AXIS:
            HAL_TIM_GenerateEvent(timDMAParamsPulse->htim, TIM_EVENTSOURCE_CC1);
            __HAL_TIM_ENABLE(timDMAParamsPulse->htim);
            break;
        case NUM_DIMENSIONS:
        default:
            break;
        }
    }

    // compute increment for pulse
    increment = COMPUTE_INCREMENT_FOR_PULSE(PULSE_FREQ, APB1_TIMER_CLOCK);

    // Infinite loop
    for (;;)
    {
        // update buffer for each axis
        for (uint8_t i = 0; i < NUM_DIMENSIONS; i++)
        {
            // check if buffer shall be updated
            doubleBufferArray_t *buf = (doubleBufferArray_t *)stepGetFreeDataAddress(i);

            if (buf == 0)
            {
                continue;
            }

            // get timer and DMA parameters of this axis
            TIM_DMA_Parameters_t *timDMAParamsPulse = &axisTimerDMAParams[i];

            // set PD4 to high ===> signal the start of updating buffer
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

            // update buffer
            stepUpdateBuffer((doubleBufferArray_t *)buf, timDMAParamsPulse->lastCounterValue + offStatePulseWidth, increment, DOUBLE_BUFFER_SIZE);

            // increment ring buffer head
            stepRingBufferIncrementHead(i);

            // set last counter value
            timDMAParamsPulse->lastCounterValue = (*buf)[DOUBLE_BUFFER_SIZE - 1];

            // set batch update threshold
            if (i == 0)
                nextUpdateBatchThreshold = timDMAParamsPulse->lastCounterValue;
            // set PD4 to low ===> signal the end of updating buffer
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
        }

        // check if there is a notification to resume DMA stream asking from the timer interrupt
        if (generalNotification & GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE)
        {
            // set PD5 to high ===> signal the start of resuming DMA stream from task
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);

            // set PD5 to low ===> signal the end of resuming DMA stream from task
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
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
    // TIM_STOP_COUNTER(MASTER_TIM); // timer x axis

    // // set PD6 to high ===> signal the start of ISR
    // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);

    // // read capture compare register
    // captureCompareReg = TIM_GET_COMPARE(htim, htim->Channel);

    // // check if the capture compare register is equal to the batch update threshold
    // if (captureCompareReg != updateBatchThreshold)
    // {
    //     // restart counter
    //     TIM_START_COUNTER(MASTER_TIM);

    //     // set PD5 to low ===> signal the end of ISR
    //     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
    //     return;
    // }

    // // suspend DMA stream and clear interrupt flag
    // for (uint8_t i = 0; i < NUM_DIMENSIONS; i++)
    // {
    //     // get timer and DMA parameters of this axis
    //     TIM_DMA_Parameters_t *timDMAParamsPulse = &axisTimerDMAParams[i].pulse;
    //     TIM_DMA_Parameters_t *timDMAParamsDir = &axisTimerDMAParams[i].dir;

    //     // suspend DMA stream
    //     SUSPEND_DMA_STREAM(timDMAParamsPulse->htim->hdma[timDMAParamsPulse->TIM_DMA_ID]);
    //     SUSPEND_DMA_STREAM(timDMAParamsDir->htim->hdma[timDMAParamsDir->TIM_DMA_ID]);

    //     // clear all DMA interrupt flags
    //     CLEAR_DMA_IT(timDMAParamsPulse->htim->hdma[timDMAParamsPulse->TIM_DMA_ID]);
    //     CLEAR_DMA_IT(timDMAParamsDir->htim->hdma[timDMAParamsDir->TIM_DMA_ID]);
    // }

    // // check if counter overruns the batch update threshold
    // uint32_t counter = __HAL_TIM_GET_COUNTER(MASTER_TIM_HANDLE);
    // int32_t diff = counter - updateBatchThreshold;
    // if ((diff > 0) && (diff < HALF_PERIOD))
    //     overrun = 1;
    // else if ((diff < 0) && (diff < -HALF_PERIOD))
    //     overrun = 1;
    // else
    //     overrun = 0;

    // // check if the preparing buffer is ready
    // if (generalNotification & GENERAL_NOTIFICATION_DATA_READY)
    // {
    //     // set PD7 to high ===> signal the start of updating DMA buffer
    //     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

    //     // disable output compare channel
    //     // TIM_CCxChannelCmd(Z_AXIS_TIM, Z_AXIS_PULSE_TIM_CHANNEL, TIM_CCx_DISABLE);

    //     // force OC1 to high
    //     // reset OC1M bits
    //     // Z_AXIS_TIM->CCMR1 &= ~TIM_CCMR1_OC1M;
    //     // set OC1M bits to 0101
    //     // Z_AXIS_TIM->CCMR1 |= TIM_OCMODE_FORCED_ACTIVE;

    //     for (uint8_t i = 0; i < NUM_DIMENSIONS; i++)
    //     {
    //         // get timer and DMA parameters of this axis
    //         TIM_DMA_Parameters_t *timDMAParamsPulse = &axisTimerDMAParams[i].pulse;
    //         TIM_DMA_Parameters_t *timDMAParamsDir = &axisTimerDMAParams[i].dir;

    //         // swap buffer
    //         SWAP_BUFFER_UINT32(timDMAParamsPulse->preparedBuffer, timDMAParamsPulse->preparingBuffer);
    //         SWAP_BUFFER_UINT32(timDMAParamsDir->preparedBuffer, timDMAParamsDir->preparingBuffer);

    //         // check if counter matches compare value
    //         if (overrun)
    //         {
    //             // set counter to capture compare register + 1
    //             __HAL_TIM_SET_COUNTER(timDMAParamsPulse->htim, captureCompareReg + 1);
    //         }

    //         // set compare value to the first element of the buffer in timer
    //         // __HAL_TIM_SET_COMPARE(timDMAParamsPulse->htim, timDMAParamsPulse->TIM_CHANNEL, (*timDMAParamsPulse->preparedBuffer)[0]);

    //         // resume DMA stream with updated buffer and length
    //         // RESUME_DMA_STREAM_WITH_TC(timDMAParamsPulse->htim->hdma[timDMAParamsPulse->TIM_DMA_ID], ((uint32_t *)(timDMAParamsPulse->preparedBuffer) + 1), timDMAParamsPulse->lengthAvailableData - 1);
    //         RESUME_DMA_STREAM_WITH_TC(timDMAParamsPulse->htim->hdma[timDMAParamsPulse->TIM_DMA_ID], timDMAParamsPulse->preparedBuffer, timDMAParamsPulse->lengthAvailableData);
    //         RESUME_DMA_STREAM_WITH_TC(timDMAParamsDir->htim->hdma[timDMAParamsDir->TIM_DMA_ID], timDMAParamsDir->preparedBuffer, timDMAParamsDir->lengthAvailableData);
    //     }
    //     // set batch update threshold with the next batch update threshold
    //     updateBatchThreshold = nextUpdateBatchThreshold;

    //     // restart counter
    //     TIM_START_COUNTER(MASTER_TIM);

    //     // reset the notification
    //     generalNotification &= ~GENERAL_NOTIFICATION_DATA_READY;

    //     // set PD7 to low ===> signal the end of updating DMA buffer
    //     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
    // }
    // else
    // {
    //     // notify main task to update pulse data
    //     generalNotification |= GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE;
    // }

    // // set PD6 to low ===> signal the end of ISR
    // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
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
        TIM_STOP_COUNTER(&MASTER_TIM_HANDLE); // timer x axis

        // set notification to data not available
        generalNotification |= GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE;

        return;
    }

    if(axis == Y_AXIS)
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
        TIM_STOP_COUNTER(&MASTER_TIM_HANDLE); // timer x axis

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
