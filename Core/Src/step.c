#include <string.h>
#include "stm32f7xx_hal.h"
#include "stm32f7xx_grbl.h"
#include "stm32f7xx_timer_extension.h"
#include "FreeRTOS.h"
#include "task.h"
#include "grbl.h"

#undef X_AXIS
#undef Y_AXIS
#undef Z_AXIS
#undef N_AXIS

#define N_AXIS NUM_DIMENSIONS
#define PULSE_FREQ 100000  // Hz
#define HALF_PERIOD 0x7FFF // 0xFFFF / 2
#define RING_BUFFER_SIZE 6
#define GAP_HEAD_TAIL 1 // gap between head and tail, two buffers in used by DMA
#define STEP_INCREMENT_PERIOD APB1_TIMER_CLOCK / 100
#define STEP_INCREMENT 3

/* extern variables */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;

/**
 * Type Defines
 */
typedef enum AxisEnum
{
    X_AXIS = 0,
    Y_AXIS,
    Z_AXIS,
    NUM_DIMENSIONS
} axis_t;

typedef struct TIM_DMA_Parameters
{
    TIM_HandleTypeDef *htim;
    uint8_t TIM_DMA_ID;
    uint8_t TIM_DMA_ID_CC;
    uint8_t TIM_CHANNEL;
    HAL_TIM_ActiveChannel TIM_ACTIVE_CHANNEL;
    uint32_t CCRx_Addr;
    uint32_t CompareEventID;
    uint8_t Step_Bit;
    uint8_t Dir_Bit;
} TIM_DMA_Parameters_t;

typedef struct
{
    doubleBufferArray_t data;
    uint8_t dir_outbit; // output bit is shifted by the pin number defined in cpu_map.h
    uint16_t length;
    uint8_t motion_control_state;
} pulse_t;

/*
 * Type defines copied from stepper.c
 */
typedef uint16_t IO_TYPE;

// Stores the planner block Bresenham algorithm execution data for the segments in the segment
// buffer. Normally, this buffer is partially in-use, but, for the worst case scenario, it will
// never exceed the number of accessible stepper buffer segments (SEGMENT_BUFFER_SIZE-1).
// NOTE: This data is copied from the prepped planner blocks so that the planner blocks may be
// discarded when entirely consumed and completed by the segment buffer. Also, AMASS alters this
// data for its own use.
typedef struct
{
    uint32_t steps[N_AXIS];
    uint32_t step_event_count;
    IO_TYPE direction_bits;
#ifdef ENABLE_DUAL_AXIS
    uint8_t direction_bits_dual;
#endif
#ifdef VARIABLE_SPINDLE
    uint8_t is_pwm_rate_adjusted; // Tracks motions that require constant laser power/rate
#endif
} st_block_t;

// Primary stepper segment ring buffer. Contains small, short line segments for the stepper
// algorithm to execute, which are "checked-out" incrementally from the first block in the
// planner buffer. Once "checked-out", the steps in the segments buffer cannot be modified by
// the planner, where the remaining planner block steps still can.
typedef struct
{
    uint16_t n_step;          // Number of step events to be executed for this segment
    uint16_t cycles_per_tick; // Step distance traveled per ISR tick, aka step rate.
    uint8_t st_block_index;   // Stepper block data index. Uses this information to execute this segment.
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint8_t amass_level; // Indicates AMASS level for the ISR to execute this segment
#else
    uint8_t prescaler; // Without AMASS, a prescaler is required to adjust for slow timing.
#endif
#ifdef VARIABLE_SPINDLE
    uint8_t spindle_pwm;
#endif
} segment_t;

// Stepper ISR data struct. Contains the running data for the main stepper ISR.
typedef struct
{
    // Used by the bresenham line algorithm
    uint32_t counter_x, // Counter variables for the bresenham line tracer
        counter_y,
        counter_z;
#ifdef STEP_PULSE_DELAY
    uint8_t step_bits; // Stores out_bits output to complete the step pulse delay
#endif

    uint8_t execute_step;    // Flags step execution for each interrupt.
    uint8_t step_pulse_time; // Step pulse reset time after step rise
    IO_TYPE step_outbits;    // The next stepping-bits to be output
    IO_TYPE dir_outbits;
#ifdef ENABLE_DUAL_AXIS
    uint8_t step_outbits_dual;
    uint8_t dir_outbits_dual;
#endif
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint32_t steps[N_AXIS];
#endif

    uint16_t step_count;      // Steps remaining in line segment motion
    uint8_t exec_block_index; // Tracks the current st_block index. Change indicates new block.
    st_block_t *exec_block;   // Pointer to the block data for the segment being executed
    segment_t *exec_segment;  // Pointer to the segment being executed
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

// declare an array of general notification
volatile uint32_t generalNotification;

// current steppers state, which can be used to determine if a stepper shall be re-enabled from idle.
volatile uint8_t currentStepperState = 0; // bit 0: x axis active, bit 1: y axis active, bit 2: z axis active

// record the axes that their DMA buffer has been updated
volatile uint8_t DMAUpdatedAxes = 0; // bit 0: x axis, bit 1: y axis, bit 2: z axis

// current counter value only for calculating the pulse data
volatile uint32_t currentCounterValue = MINIMUN_LOW_PULSE_WIDTH_TICKS;

// handle for step task
TaskHandle_t xHandleStepTask = NULL;

/**
 * Function Prototypes
 */
void stepTask(void *pvParameters);
void stepUpdateDMABuffer(TIM_HandleTypeDef *htim, uint32_t address, axis_t axis);
static void TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma);
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

    // initialize ring buffer head and tail
    ringBufferHead[X_AXIS] = 0;
    ringBufferTail[X_AXIS] = 0;
    ringBufferHead[Y_AXIS] = 0;
    ringBufferTail[Y_AXIS] = 0;
    ringBufferHead[Z_AXIS] = 0;
    ringBufferTail[Z_AXIS] = 0;

    // initialize general notification
    generalNotification = 0;

    // initialize current stepper state
    currentStepperState = 0;

    // initialize completed DMA transfer axes
    DMAUpdatedAxes = 0;

    // initialize current counter value
    currentCounterValue = MINIMUN_LOW_PULSE_WIDTH_TICKS;

    // check if task is created
    if (xHandleStepTask != NULL)
    {
        // delete the task
        vTaskDelete(xHandleStepTask);

        // set the handle to NULL
        xHandleStepTask = NULL;
    }

    // create tasks
    xTaskCreate(stepTask, "StepTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandleStepTask);
}

// handler for step task
void stepTask(void *pvParameters)
{
    // set PD3 to high to enable level shifter
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);

    // set general notification to data not available to start the process
    generalNotification |= (GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE_ALL_AXES | GENERAL_NOTIFICATION_FIRST_TIME_START);

    // Infinite loop
    for (;;)
    {
        // prepare pulse data
        if ((generalNotification & GENERAL_NOTIFICATION_CALCULATE_PULSE))
        {
            // calculate pulse data
            stepper_pulse_generation_isr();
        }

        // check if there is a notification to resume DMA stream asking from the timer interrupt
        if (!(generalNotification & GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE_ALL_AXES))
        {
            continue;
        }

        // check if there is any buffer available
        if (stepRingBufferGetTail(Z_AXIS) == UINT16_MAX)
        {
            continue;
        }

        // check if it is the "First Time" to start the process
        if (generalNotification & GENERAL_NOTIFICATION_FIRST_TIME_START)
        {
            // clear the flag
            generalNotification &= ~GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE_ALL_AXES;
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
                uint8_t dirOutputBit = GET_DIRECTION_BIT_FROM_AXIS(i);
                volatile uint32_t *pDirOutputPort = GET_DIRECTION_PORT_FROM_AXIS(i);

                // set the direction output bit according to the dir_outbit
                *pDirOutputPort = (*pDirOutputPort & ~(1 << dirOutputBit)) | (((pulse_t *)address)->dir_outbit);

                // start timer output mode with DMA stream
                stepTimeOCStartDMA(timDMAParamsPulse->htim, timDMAParamsPulse->TIM_CHANNEL, (uint32_t *)address, ((pulse_t *)address)->length);

                // determine if this axis is active or not by checking motion_control_state coming with the pulse data
                if (((pulse_t *)address)->motion_control_state & (1 << i))
                {
                    // set the axis to active
                    currentStepperState |= (1 << i);

                    // generate compare event
                    HAL_TIM_GenerateEvent(timDMAParamsPulse->htim, timDMAParamsPulse->CompareEventID);

                    // check if it is the master timer
                    if (timDMAParamsPulse->htim != &MASTER_TIM_HANDLE)
                    {
                        // enable the timer
                        __HAL_TIM_ENABLE(timDMAParamsPulse->htim);
                    }
                }
                else // this axis is not active
                {
                    // set the axis to idle
                    currentStepperState &= ~(1 << i);

                    // force output pin to low in output compare mode
                    FORCE_OC_OUTPUT_LOW(timDMAParamsPulse->htim, timDMAParamsPulse->TIM_ACTIVE_CHANNEL);
                }
            }

            // enable the master timer
            __HAL_TIM_ENABLE(&MASTER_TIM_HANDLE);
        }
        // NOT the "First Time" to start the process, but out of data asking from the timer interrupt
        else
        {
            // set PD5 to high ===> signal the start of resuming DMA stream
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);

            // set up each axis
            for (uint8_t i = 0; i < NUM_DIMENSIONS; i++)
            {
                // determine which axis notified that it is out of data
                if (generalNotification & GET_DATA_NOT_AVAILABLE_BIT(i))
                {
                    // get available data address
                    uint32_t address = stepGetAvailableDataAddress(i);

                    // check if the address is valid
                    if (address != 0)
                    {
                        // update DMA buffer
                        stepUpdateDMABuffer(axisTimerDMAParams[i].htim, address, i);

                        // reset the flag in general notification
                        generalNotification &= ~GET_DATA_NOT_AVAILABLE_BIT(i);

                        // restart counter
                        TIM_START_COUNTER(MASTER_TIM_HANDLE);
                    }
                    else
                    {
                        // should not reach here, because the 3-axis data is calculated simultaneously
                        Error_Handler();
                    }
                }
            }

            // set PD5 to low ===> signal the end of resuming DMA stream
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
        }
    }
}

HAL_StatusTypeDef stepCalculatePulseData(uint32_t *st_addr)
{
    // set PD4 to high ===> signal the start of pulse calculation
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

    static uint8_t getNewBuffer = (1 << NUM_DIMENSIONS) - 1; // bit 0: x axis, bit 1: y axis, bit 2: z axis
    static pulse_t *pulse[NUM_DIMENSIONS] = {0};
    static IO_TYPE dirOutputBits = 0;

    stepper_t *st = (stepper_t *)st_addr;

    // loop through all axes
    for (uint8_t i = 0; i < NUM_DIMENSIONS; i++)
    {
        // get timer and DMA parameters of this axis
        TIM_DMA_Parameters_t *timDMAParamsPulse = &axisTimerDMAParams[i];
        uint8_t step_bit = timDMAParamsPulse->Step_Bit;
        uint8_t dir_bit = timDMAParamsPulse->Dir_Bit;

        // determine if a new buffer is needed
        if (getNewBuffer & (1 << i) || (dirOutputBits != st->dir_outbits))
        {

            // get new buffer
            pulse[i] = (pulse_t *)stepGetFreeDataAddress(i);

            // check if the buffer is available
            if (pulse[i] == 0)
            {
                // set PD4 to low ===> signal the end of pulse calculation
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);

                return HAL_BUSY;
            }

            // reset buffer length
            pulse[i]->length = 0;

            // reset motion control state
            pulse[i]->motion_control_state = 0;

            // set direction state
            pulse[i]->dir_outbit = st->dir_outbits & (1 << dir_bit);

            // reset the flag
            getNewBuffer &= ~(1 << i);

            // update direction output bits
            dirOutputBits = st->dir_outbits;
        }

        // determine if a pair of pulse data should be added in this axis or not by Bresenham line algorithm.
        // a pair of pulse data consists of a pulse data for the on state and a pulse data for the off state.
        uint16_t buf_length = pulse[i]->length;

        if (st->step_outbits & (1 << step_bit))
        {
            // set this axis to ACTIVE state
            pulse[i]->motion_control_state |= (1 << i);

            // add pulse data
            // add ON state pulse data
            pulse[i]->data[buf_length++] = currentCounterValue;
            // add OFF state pulse data
            pulse[i]->data[buf_length++] = currentCounterValue + st->step_pulse_time;

            // update the length of available data
            pulse[i]->length = buf_length;

            // check if the buffer is full
            if (buf_length >= DOUBLE_BUFFER_SIZE)
            {
                // set the flag to get new buffer
                getNewBuffer |= (1 << i);
            }
        }
    }

    // update variables
    currentCounterValue += st->exec_segment->cycles_per_tick;

    // check if any buffer is full of data
    if (getNewBuffer > 0)
    {
        // atomic operation
        __disable_irq();

        // increment head
        stepRingBufferIncrementHead(X_AXIS);
        stepRingBufferIncrementHead(Y_AXIS);
        stepRingBufferIncrementHead(Z_AXIS);

        // atomic operation
        __enable_irq();

        getNewBuffer = (1 << NUM_DIMENSIONS) - 1;
    }

    // set PD4 to low ===> signal the end of pulse calculation
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);

    return HAL_OK;
}

void stepUpdateDMABuffer(TIM_HandleTypeDef *htim, uint32_t address, axis_t axis)
{
    // set PD7 to high ===> signal the start of updating DMA buffer
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

    uint8_t dirOutputBit = GET_DIRECTION_BIT_FROM_AXIS(axis);
    volatile uint32_t *pDirOutputPort = GET_DIRECTION_PORT_FROM_AXIS(axis);
    uint8_t upcomingAxesActiveState = 0;

    // get upcoming axes active state
    upcomingAxesActiveState = ((pulse_t *)address)->motion_control_state;

    // check if this axis shall be put into idle or not
    if (!(upcomingAxesActiveState & (1 << axis))) // no pulsed data to be sent
    {
        // set the axis to idle
        currentStepperState &= ~(1 << axis);

        // force output pin to low in output compare mode
        FORCE_OC_OUTPUT_LOW(htim, axisTimerDMAParams[axis].TIM_ACTIVE_CHANNEL);
    }
    else // still has pulsed data to be sent
    {
        // set the axis to active
        currentStepperState |= (1 << axis);

        // set the direction output bit according to the dir_outbit
        *pDirOutputPort = (*pDirOutputPort & ~(1 << dirOutputBit)) | (((pulse_t *)address)->dir_outbit);

        // resume DMA stream with updated buffer and length
        RESUME_DMA_STREAM_WITH_TC(htim->hdma[GET_TIM_DMA_ID_FROM_AXIS(axis)], address, ((pulse_t *)address)->length);
    }

    // set the axis that its DMA transfer is completed
    DMAUpdatedAxes |= (1 << axis);

    /**
     * Check if there is any axis that shall be re-enabled from idle
     */
    // check if there is any bit that is set to 0 in currentStepperState
    // and the corresponding bit is set to 1 in upcomingAxesActiveState.
    // make these axes which shall be re-enabled from idle
    uint8_t axisToBeReEnabled = (currentStepperState ^ upcomingAxesActiveState) & upcomingAxesActiveState;
    if (axisToBeReEnabled)
    {
        // loop through all axes
        for (uint8_t i = 0; i < NUM_DIMENSIONS; i++)
        {
            // check if this axis shall be re-enabled from idle
            if (axisToBeReEnabled & (1 << i))
            {
                // set the axis to active
                currentStepperState |= (1 << i);

                // get available data address
                address = stepGetAvailableDataAddress(i);
                dirOutputBit = GET_DIRECTION_BIT_FROM_AXIS(i);
                pDirOutputPort = GET_DIRECTION_PORT_FROM_AXIS(i);

                // check if the address is valid
                if (address != 0)
                {
                    // set the direction output bit according to the dir_outbit
                    *pDirOutputPort = (*pDirOutputPort & ~(1 << dirOutputBit)) | (((pulse_t *)address)->dir_outbit);

                    // resume DMA stream with updated buffer and length
                    RESUME_DMA_STREAM_WITH_TC(axisTimerDMAParams[i].htim->hdma[axisTimerDMAParams[i].TIM_DMA_ID], address, ((pulse_t *)address)->length);

                    // generate compare event to trigger the DMA transfer
                    HAL_TIM_GenerateEvent(axisTimerDMAParams[i].htim, axisTimerDMAParams[i].CompareEventID);

                    // set this axis that its DMA buffer has been updated
                    DMAUpdatedAxes |= (1 << i);

                    // force output turned into toggle mode
                    FORCE_OC_OUTPUT_TOGGLE(axisTimerDMAParams[i].htim, axisTimerDMAParams[i].htim->Channel);
                }
                else
                {
                    // should not reach here, because the 3-axis data is calculated simultaneously
                    Error_Handler();
                }
            }
        }
    }

    /**
     * Update Variables
     */
    // check if all axes have completed their DMA transfer
    if (DMAUpdatedAxes == (1 << NUM_DIMENSIONS) - 1)
    {
        // reset the axes that their DMA transfer is completed
        DMAUpdatedAxes = 0;

        // set generatl notification to all axes have completed their DMA transfer
        generalNotification |= GENERAL_NOTIFICATION_ALL_AXES_DMA_UPDATED;
    }

    // set PD7 to low ===> signal the end of updating DMA buffer
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
}

/* =========================================================== */
/*                   DMA Transfer Complete ISR                 */
/* =========================================================== */
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

    // suspend DMA stream
    // ==> to update DMA buffer length and address, DMA stream should be suspended
    SUSPEND_DMA_STREAM(htim->hdma[GET_TIM_DMA_ID_FROM_AXIS(axis)]);

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

    /**
     * Start Update DMA buffer
     */
    // get the available data address
    uint32_t address = stepGetAvailableDataAddress(axis);

    // check if the address is valid
    if (address != 0)
    {
        stepUpdateDMABuffer(htim, address, axis);

        // restart counter
        TIM_START_COUNTER(MASTER_TIM_HANDLE);
    }
    else
    {
        // notify main task to update pulse data
        generalNotification |= GET_DATA_NOT_AVAILABLE_BIT(axis);
    }

    // set PD6 to low ===> signal the end of ISR
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
}

/* =========================================================== */
/*       Assistant Functions for Ring buffer management        */
/* =========================================================== */
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

/* ===================================================================== */
/*                      Everything about Stepper                         */
/* ===================================================================== */

void stepWakeUp()
{
    // check if pulse calculation is disabled
    if (generalNotification & GENERAL_NOTIFICATION_FORCE_STOP)
    {
        // clear Force stop flag
        generalNotification &= ~GENERAL_NOTIFICATION_FORCE_STOP;

        // start counter
        TIM_START_COUNTER(MASTER_TIM_HANDLE); // timer x axis
    }

    // start pulse calculation
    stepEnablePulseCalculate();
}

void stepGoIdle()
{
    // stop counter
    TIM_STOP_COUNTER(MASTER_TIM_HANDLE); // timer x axis

    // set Force stop flag
    generalNotification |= GENERAL_NOTIFICATION_FORCE_STOP;

    // stop pulse calculation
    stepDisablePulseCalculate();
}

/**
 * @brief  Enable pulse calculation
 */
void stepEnablePulseCalculate()
{
    // clear general notification
    generalNotification |= GENERAL_NOTIFICATION_CALCULATE_PULSE;
}

/**
 * @brief  Disable pulse calculation
 */
void stepDisablePulseCalculate()
{
    // set general notification to no more segment which is triggered by stepper.c in grbl
    generalNotification &= ~GENERAL_NOTIFICATION_CALCULATE_PULSE;
}

/* ===================================================================== */
/*                      Everything about Timer                           */
/* ===================================================================== */

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
