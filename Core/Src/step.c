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
#define RING_BUFFER_SIZE 16
#define GAP_HEAD_TAIL 3 // gap between head and tail, two buffers in used by DMA
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
    uint32_t TIM_FLAG_CCx; // capture/compare interrupt flag
} TIM_DMA_Parameters_t;

typedef struct
{
    doubleBufferArray_t data;
    uint16_t length;
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
    uint32_t cycles_per_tick; // Step distance traveled per ISR tick, aka step rate.
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

    uint8_t execute_step;     // Flags step execution for each interrupt.
    uint32_t step_pulse_time; // Step pulse reset time after step rise
    IO_TYPE step_outbits;     // The next stepping-bits to be output
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

typedef struct
{
    pulse_t pulse_data[NUM_DIMENSIONS];
    uint8_t motion_control_state;
    IO_TYPE dir_outbits;
} pulse_block_t;

/**
 * Declare Variables
 */
// Ring buffer for each axis
static pulse_block_t pulseRingBuffer[RING_BUFFER_SIZE] = {0};

// ring buffer head and tail for each axis
volatile uint16_t pulseRingBufferHead = 0;
volatile uint16_t pulseRingBufferTail = 0;

// declare an array of timer and DMA parameters for each axis
static TIM_DMA_Parameters_t axisTimerDMAParams[NUM_DIMENSIONS];

// declare an array of general notification
volatile uint32_t generalNotification;

// current steppers state, which can be used to determine if a stepper shall be re-enabled from idle.
volatile uint8_t currentStepperState = 0; // bit 0: x axis active, bit 1: y axis active, bit 2: z axis active

// record the axes that their DMA buffer has been updated
volatile uint8_t DMAUpdatedAxes = 0; // bit 0: x axis, bit 1: y axis, bit 2: z axis

// track DMA transfer completion
volatile uint8_t DMATransferCompletedAxes = 0; // bit 0: x axis, bit 1: y axis, bit 2: z axis

// current counter value only for calculating the pulse data
volatile uint32_t currentCounterValue = MINIMUN_LOW_PULSE_WIDTH_TICKS;

volatile uint32_t pulseBlockAddress = 0;

// handle for step task
TaskHandle_t xHandleStepTask = NULL;

/**
 * Function Prototypes
 */
void stepTask(void *pvParameters);
void stepUpdateDMABuffer(uint32_t address);
static void TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef stepTimeOCStartDMA(TIM_HandleTypeDef *htim, uint32_t Channel, const uint32_t *pData, uint16_t Length);
uint16_t stepRingBufferGetNext();
void stepRingBufferIncrementHead();
uint16_t stepRingBufferGetTail();
void stepRingBufferIncrementTail();
uint32_t stepGetFreeDataAddress();
uint32_t stepGetAvailableDataAddress();
void stepSetTimerOC1Mode(TIM_TypeDef *TIMx, const uint32_t oc_mode);
void stepSetTimerOC2Mode(TIM_TypeDef *TIMx, const uint32_t oc_mode);
void stepSetTimerOC3Mode(TIM_TypeDef *TIMx, const uint32_t oc_mode);
void stepSetTimerOC4Mode(TIM_TypeDef *TIMx, const uint32_t oc_mode);

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
    pulseRingBufferHead = 0;
    pulseRingBufferTail = 0;

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
    xTaskCreate(stepTask, "StepTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &xHandleStepTask);
}

// handler for step task
void stepTask(void *pvParameters)
{
    // set PD3 to high to enable level shifter
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);

    // set general notification to data not available to start the process
    generalNotification |= (GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE_ALL_AXES | GENERAL_NOTIFICATION_FIRST_TIME_START);

    // wait notification to calculate pulse data
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Infinite loop
    for (;;)
    {
        // prepare pulse data
        if ((generalNotification & GENERAL_NOTIFICATION_CALCULATE_PULSE))
        {
            // calculate pulse data
            stepper_pulse_generation_isr();
        }
        else
        {
            // inform to get new free buffer
            generalNotification |= GENERAL_NOTIFICATION_GET_NEW_BUFFER;

            pulse_block_t *pulseBlock = &(pulseRingBuffer[pulseRingBufferHead]);

            // check if the pulse block is not empty
            // in case appending empty pulse block to DMA
            if (pulseBlock->motion_control_state)
            {
                // increment head
                stepRingBufferIncrementHead();

                // clear motion control state
                pulseRingBuffer[pulseRingBufferHead].motion_control_state = 0;
            }

            // wait notification to calculate pulse data
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }

        // check if there is a notification to resume DMA stream asking from the timer interrupt
        if (!(generalNotification & GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE_ALL_AXES))
        {
            continue;
        }

        // check if there is any buffer available
        if (stepRingBufferGetTail() == UINT16_MAX)
        {
            continue;
        }

        // check if it is the "First Time" to start the process
        if (generalNotification & GENERAL_NOTIFICATION_FIRST_TIME_START)
        {
            // get available data address
            pulseBlockAddress = stepGetAvailableDataAddress();
            pulse_block_t *pulseBlock = (pulse_block_t *)pulseBlockAddress;

            // set up each axis
            for (int8_t i = NUM_DIMENSIONS - 1; i >= 0; i--)
            {
                // get pulse data
                pulse_t *pulseData = &(pulseBlock->pulse_data[i]);

                // get timer and DMA parameters of this axis
                TIM_DMA_Parameters_t *timDMAParamsPulse = &axisTimerDMAParams[i];

                // reset timer
                __HAL_TIM_SetCounter(timDMAParamsPulse->htim, 0);

                uint8_t dirOutputBit = GET_DIRECTION_BIT_FROM_AXIS(i);
                volatile uint32_t *pDirOutputPort = GET_DIRECTION_PORT_FROM_AXIS(i);

                // set the direction output bit according to the dir_outbit
                *pDirOutputPort = (*pDirOutputPort & ~(1 << dirOutputBit)) | (pulseBlock->dir_outbits & (1 << dirOutputBit));

                // determine if this axis is active or not by checking motion_control_state coming with the pulse data
                if (pulseBlock->motion_control_state & (1 << i))
                {
                    // start timer output mode with DMA stream
                    stepTimeOCStartDMA(timDMAParamsPulse->htim, timDMAParamsPulse->TIM_CHANNEL, (uint32_t *)pulseData, pulseData->length);

                    // generate compare event
                    HAL_TIM_GenerateEvent(timDMAParamsPulse->htim, timDMAParamsPulse->CompareEventID);
                }
                else // this axis is not active
                {
                    // force output pin to low in output compare mode
                    FORCE_OC_OUTPUT_LOW(timDMAParamsPulse->htim, timDMAParamsPulse->TIM_CHANNEL);
                    // stepSetTimerOC2Mode(timDMAParamsPulse->htim->Instance, TIM_OCMODE_INACTIVE);
                }

                // check if it is the master timer
                if (timDMAParamsPulse->htim != &MASTER_TIM_HANDLE)
                {
                    // enable the timer
                    __HAL_TIM_ENABLE(timDMAParamsPulse->htim);
                }
            }

            // update current motion control state by the motion_control_state coming with the pulse data
            currentStepperState = pulseBlock->motion_control_state;

            // set all DMA updated flag
            generalNotification |= GENERAL_NOTIFICATION_ALL_AXES_DMA_UPDATED;

            // enable the master timer
            __HAL_TIM_ENABLE(&MASTER_TIM_HANDLE);

            // clear the flag
            generalNotification &= ~GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE_ALL_AXES;
            generalNotification &= ~GENERAL_NOTIFICATION_FIRST_TIME_START;
        }
        // NOT the "First Time" to start the process, but out of data asking from the timer interrupt
        else
        {
            // set PD5 to high ===> signal the start of resuming DMA stream
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);

            // Determine if it should request for new data or not.
            if (currentStepperState == DMATransferCompletedAxes)
            {
                // get available address
                pulseBlockAddress = stepGetAvailableDataAddress();

                if (pulseBlockAddress == 0)
                {
                    // Error_Handler();
                    continue;
                }

                // clear DMA transfer completed axes
                DMATransferCompletedAxes = 0;
            }

            // update DMA buffer
            stepUpdateDMABuffer(pulseBlockAddress);

            // restart counter
            TIM_START_COUNTER(MASTER_TIM_HANDLE);

            // clear the flag
            generalNotification &= ~GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE_ALL_AXES;

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
    static pulse_block_t *pulseBlock = {0};
    static pulse_t *pulse;
    static IO_TYPE dirOutputBits = 0;
    static uint32_t cycles_per_tick = 0;
    // static uint8_t amass_level = 0;

    stepper_t *st = (stepper_t *)st_addr;

    if (st->exec_segment)
    {
        // update variables
        cycles_per_tick = st->exec_segment->cycles_per_tick;
        // amass_level = st->exec_segment->amass_level;
    }

    if (!(st->step_pulse_time) || (st->exec_segment->cycles_per_tick == 0) || (st->exec_segment->cycles_per_tick == 0xffffffff))
    {
        // set PD4 to low ===> signal the end of pulse calculation
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);

        return HAL_ERROR;
    }

    // determine if a new buffer is needed
    if (getNewBuffer || (dirOutputBits != st->dir_outbits) || (generalNotification & GENERAL_NOTIFICATION_GET_NEW_BUFFER))
    {
        // get new buffer from the ring buffer
        pulseBlock = (pulse_block_t *)stepGetFreeDataAddress();

        // check if the buffer is available
        if (pulseBlock == 0)
        {
            // set PD4 to low ===> signal the end of pulse calculation
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);

            return HAL_BUSY;
        }

        // reset motion control state
        pulseBlock->motion_control_state = 0;

        // set direction state
        pulseBlock->dir_outbits = st->dir_outbits;

        // update direction output bits
        dirOutputBits = st->dir_outbits;

        // loop through all axes
        for (uint8_t i = 0; i < NUM_DIMENSIONS; i++)
        {
            // get pulse data
            pulse = &(pulseBlock->pulse_data[i]);

            // clear pulse length
            pulse->length = 0;
        }

        // clear the flag
        generalNotification &= ~GENERAL_NOTIFICATION_GET_NEW_BUFFER;
        getNewBuffer = 0;
    }

    // loop through all axes
    for (uint8_t i = 0; i < NUM_DIMENSIONS; i++)
    {
        // get timer and DMA parameters of this axis
        TIM_DMA_Parameters_t *timDMAParamsPulse = &axisTimerDMAParams[i];
        uint8_t step_bit = timDMAParamsPulse->Step_Bit;

        // get pulse data
        pulse = &(pulseBlock->pulse_data[i]);

        // determine if a pair of pulse data should be added in this axis or not by Bresenham line algorithm.
        // a pair of pulse data consists of a pulse data for the on state and a pulse data for the off state.
        uint16_t buf_length = pulse->length;

        if (st->step_outbits & (1 << step_bit))
        {
            // set this axis to ACTIVE state
            pulseBlock->motion_control_state |= (1 << i);

            // add pulse data
            // add ON state pulse data
            pulse->data[buf_length++] = currentCounterValue;
            // add OFF state pulse data
            pulse->data[buf_length++] = currentCounterValue + st->step_pulse_time;

            // update the length of available data
            pulse->length = buf_length;

            // check if the buffer is full
            if (buf_length >= DOUBLE_BUFFER_SIZE)
            {
                // set the flag to get new buffer
                getNewBuffer |= (1 << i);
            }
        }
    }

    // update variables
    currentCounterValue += cycles_per_tick; // * (amass_level + 1);

    // check if any buffer is full of data
    if (getNewBuffer)
    {
        // increment head
        stepRingBufferIncrementHead();

        // clear motion control state
        pulseRingBuffer[pulseRingBufferHead].motion_control_state = 0;

    }

    // set PD4 to low ===> signal the end of pulse calculation
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);

    return HAL_OK;
}

void stepUpdateDMABuffer(uint32_t address)
{
    // set PD7 to high ===> signal the start of updating DMA buffer
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

    pulse_block_t *pulseBlock = (pulse_block_t *)address;
    uint8_t upcomingAxesActiveState = pulseBlock->motion_control_state; // get upcoming axes active state
    // check if there is any bit that is set to 0 in currentStepperState
    // and the corresponding bit is set to 1 in upcomingAxesActiveState.
    // make these axes which shall be re-enabled from idle
    uint8_t axisToBeReEnabled = (currentStepperState ^ upcomingAxesActiveState) & upcomingAxesActiveState;

    /**
     * Check if there is any axis that shall be re-enabled from idle, resume DMA stream, or turn into idle state
     */
    // manual update those axes that shall be re-enabled from idle and will stay in idle state in the upcoming cycle
    // to avoid update the same axis twice,
    // loop through all axes
    for (uint8_t i = 0; i < NUM_DIMENSIONS; i++)
    {
        // pulse data
        pulse_t *pulse = &(pulseBlock->pulse_data[i]);
        uint8_t dirOutputBit = GET_DIRECTION_BIT_FROM_AXIS(i);
        volatile uint32_t *pDirOutputPort = GET_DIRECTION_PORT_FROM_AXIS(i);

        // check if this axis shall be re-enabled from idle
        if (axisToBeReEnabled & (1 << i))
        {
            // set the axis to active
            currentStepperState |= (1 << i);

            // check if the address is valid
            if (address != 0)
            {
                // set the direction output bit according to the dir_outbit
                *pDirOutputPort = (*pDirOutputPort & ~(1 << dirOutputBit)) | (pulseBlock->dir_outbits & (1 << dirOutputBit));

                // force output turned into toggle mode
                FORCE_OC_OUTPUT_TOGGLE(axisTimerDMAParams[i].htim, axisTimerDMAParams[i].TIM_CHANNEL);

                // resume DMA stream with updated buffer and length
                // RESUME_DMA_STREAM_WITH_TC(axisTimerDMAParams[i].htim->hdma[axisTimerDMAParams[i].TIM_DMA_ID], (uint32_t *)pulse, pulse->length);
                HAL_StatusTypeDef status = stepTimeOCStartDMA(axisTimerDMAParams[i].htim, axisTimerDMAParams[i].TIM_CHANNEL, (uint32_t *)pulse, pulse->length);

                if (status == HAL_BUSY)
                {
                    RESUME_DMA_STREAM_WITH_TC(axisTimerDMAParams[i].htim->hdma[axisTimerDMAParams[i].TIM_DMA_ID], (uint32_t *)pulse, pulse->length);
                }

                // check if the capture/compare interrupt flag is set
                if (__HAL_TIM_GET_FLAG(axisTimerDMAParams[i].htim, axisTimerDMAParams[i].TIM_FLAG_CCx) == RESET)
                {
                    // generate compare event to trigger the DMA transfer
                    HAL_TIM_GenerateEvent(axisTimerDMAParams[i].htim, axisTimerDMAParams[i].CompareEventID);
                }
            }
            else
            {
                // should not reach here, because the 3-axis data is calculated simultaneously
                Error_Handler();
            }
        }
        else if ((currentStepperState & upcomingAxesActiveState) & (1 << i)) // active mode(present) -> active mode(upcoming)
        {
            // still in active state

            // resume the DMA stream
            // set the axis to active
            currentStepperState |= (1 << i);

            // set the direction output bit according to the dir_outbit
            *pDirOutputPort = (*pDirOutputPort & ~(1 << dirOutputBit)) | (pulseBlock->dir_outbits & (1 << dirOutputBit));

            // resume DMA stream with updated buffer and length
            HAL_StatusTypeDef status = stepTimeOCStartDMA(axisTimerDMAParams[i].htim, axisTimerDMAParams[i].TIM_CHANNEL, (uint32_t *)pulse, pulse->length);
            if (status == HAL_BUSY)
            {
                RESUME_DMA_STREAM_WITH_TC(axisTimerDMAParams[i].htim->hdma[GET_TIM_DMA_ID_FROM_AXIS(i)], (uint32_t *)pulse, pulse->length);
            }
        }
        else // idle mode(present) -> idle mode(upcoming) or active mode(present) -> idle mode(upcoming)
        {
            // turn into idle state

            // set the axis to idle
            currentStepperState &= ~(1 << i);

            // force output pin to low in output compare mode
            FORCE_OC_OUTPUT_LOW(axisTimerDMAParams[i].htim, axisTimerDMAParams[i].TIM_CHANNEL);
        }
    }

    /**
     * Update Variables
     */

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

    // set the axis that its DMA transfer is completed
    DMATransferCompletedAxes |= (1 << axis);

    /**
     * Start Update DMA buffer
     */
    // determine if it should request for new data or not
    // if (generalNotification & GENERAL_NOTIFICATION_ALL_AXES_DMA_UPDATED)
    if (currentStepperState == DMATransferCompletedAxes)
    {
        // get the available data address
        pulseBlockAddress = stepGetAvailableDataAddress();

        // check if the address is valid
        if (pulseBlockAddress != 0)
        {
            stepUpdateDMABuffer(pulseBlockAddress);

            // clear DMA transfer completed axes
            DMATransferCompletedAxes = 0;

            // restart counter
            TIM_START_COUNTER(MASTER_TIM_HANDLE);
        }
        else
        {
            // notify main task to update pulse data
            generalNotification |= GET_DATA_NOT_AVAILABLE_BIT(axis);
        }
    }
    else
    {
        // resume counter
        TIM_START_COUNTER(MASTER_TIM_HANDLE); // timer x axis
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
uint16_t stepRingBufferGetNext()
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
    uint16_t bodyLength = (pulseRingBufferHead - pulseRingBufferTail + RING_BUFFER_SIZE) % RING_BUFFER_SIZE;

    if (bodyLength < RING_BUFFER_SIZE - GAP_HEAD_TAIL)
    {
        return (pulseRingBufferHead + 1) % RING_BUFFER_SIZE;
    }
    else
        return UINT16_MAX;
}

void stepRingBufferIncrementHead()
{
    uint16_t nextIndex = stepRingBufferGetNext();

    if (nextIndex != UINT16_MAX)
    {
        pulseRingBufferHead = nextIndex;
    }
}

uint16_t stepRingBufferGetTail()
{
    if (pulseRingBufferTail != pulseRingBufferHead)
    {
        return pulseRingBufferTail;
    }
    else
        return UINT16_MAX;
}

void stepRingBufferIncrementTail()
{
    if (pulseRingBufferTail != pulseRingBufferHead)
    {
        pulseRingBufferTail = (pulseRingBufferTail + 1) % (RING_BUFFER_SIZE);
    }
}

/**
 * Get the free data address
 * @param axis Axis to get the free data address
 * @return Free data address; 0 if no data available
 */
uint32_t stepGetFreeDataAddress()
{
    if (stepRingBufferGetNext() == UINT16_MAX)
        return 0;

    return (uint32_t)(&pulseRingBuffer[pulseRingBufferHead]);
}

/**
 * Get the available data address
 * @param axis Axis to get the available data address
 * @return Available data address; 0 if no data available
 */
uint32_t stepGetAvailableDataAddress()
{
    uint16_t tail = stepRingBufferGetTail();

    if (tail == UINT16_MAX)
        return 0;

    stepRingBufferIncrementTail();

    return (uint32_t)(&pulseRingBuffer[tail]);
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

        if (!(generalNotification & GENERAL_NOTIFICATION_FIRST_TIME_START))
        {
            // start counter
            TIM_START_COUNTER(MASTER_TIM_HANDLE); // timer x axis
        }
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

    stepNotifyContinuePulseCalculation();
}

/**
 * @brief  Disable pulse calculation
 */
void stepDisablePulseCalculate()
{
    // set general notification to no more segment which is triggered by stepper.c in grbl
    generalNotification &= ~GENERAL_NOTIFICATION_CALCULATE_PULSE;
}

void stepNotifyContinuePulseCalculation()
{
    xTaskNotifyGive(xHandleStepTask);
}

void stepSetTimerOC1Mode(TIM_TypeDef *TIMx, const uint32_t oc_mode)
{
    uint32_t tmpccmrx;
    uint32_t tmpccer;
    // uint32_t tmpcr2;

    /* Get the TIMx CCER register value */
    tmpccer = TIMx->CCER;

    /* Disable the Channel 1: Reset the CC1E Bit */
    TIMx->CCER &= ~TIM_CCER_CC1E;

    /* Get the TIMx CCMR1 register value */
    tmpccmrx = TIMx->CCMR1;

    /* Reset the Output Compare mode and Capture/Compare selection Bits */
    tmpccmrx &= ~TIM_CCMR1_OC1M;
    tmpccmrx &= ~TIM_CCMR1_CC1S;
    /* Select the Output Compare Mode */
    tmpccmrx |= oc_mode;

    /* Write to TIMx CCMR1 */
    TIMx->CCMR1 = tmpccmrx;

    /* Write to TIMx CCER */
    TIMx->CCER = tmpccer;
}

void stepSetTimerOC2Mode(TIM_TypeDef *TIMx, const uint32_t oc_mode)
{
    uint32_t tmpccmrx;
    uint32_t tmpccer;
    // uint32_t tmpcr2;

    /* Get the TIMx CCER register value */
    tmpccer = TIMx->CCER;

    /* Disable the Channel 2: Reset the CC2E Bit */
    TIMx->CCER &= ~TIM_CCER_CC2E;

    /* Get the TIMx CCMR1 register value */
    tmpccmrx = TIMx->CCMR1;

    /* Reset the Output Compare mode and Capture/Compare selection Bits */
    tmpccmrx &= ~TIM_CCMR1_OC2M;
    tmpccmrx &= ~TIM_CCMR1_CC2S;
    /* Select the Output Compare Mode */
    tmpccmrx |= (oc_mode << 8U);

    /* Write to TIMx CCMR1 */
    TIMx->CCMR1 = tmpccmrx;

    /* Write to TIMx CCER */
    TIMx->CCER = tmpccer;
}

void stepSetTimerOC3Mode(TIM_TypeDef *TIMx, const uint32_t oc_mode)
{
    uint32_t tmpccmrx;
    uint32_t tmpccer;
    // uint32_t tmpcr2;

    /* Get the TIMx CCER register value */
    tmpccer = TIMx->CCER;

    /* Disable the Channel 3: Reset the CC3E Bit */
    TIMx->CCER &= ~TIM_CCER_CC3E;

    /* Get the TIMx CCMR2 register value */
    tmpccmrx = TIMx->CCMR2;

    /* Reset the Output Compare mode and Capture/Compare selection Bits */
    tmpccmrx &= ~TIM_CCMR2_OC3M;
    tmpccmrx &= ~TIM_CCMR2_CC3S;
    /* Select the Output Compare Mode */
    tmpccmrx |= oc_mode;

    /* Write to TIMx CCMR2 */
    TIMx->CCMR2 = tmpccmrx;

    /* Write to TIMx CCER */
    TIMx->CCER = tmpccer;
}

void stepSetTimerOC4Mode(TIM_TypeDef *TIMx, const uint32_t oc_mode)
{
    uint32_t tmpccmrx;
    uint32_t tmpccer;
    // uint32_t tmpcr2;

    /* Get the TIMx CCER register value */
    tmpccer = TIMx->CCER;

    /* Disable the Channel 4: Reset the CC4E Bit */
    TIMx->CCER &= ~TIM_CCER_CC4E;

    /* Get the TIMx CCMR2 register value */
    tmpccmrx = TIMx->CCMR2;

    /* Reset the Output Compare mode and Capture/Compare selection Bits */
    tmpccmrx &= ~TIM_CCMR2_OC4M;
    tmpccmrx &= ~TIM_CCMR2_CC4S;
    /* Select the Output Compare Mode */
    tmpccmrx |= (oc_mode << 8U);

    /* Write to TIMx CR2 */
    //   TIMx->CR2 = tmpcr2;

    /* Write to TIMx CCMR2 */
    TIMx->CCMR2 = tmpccmrx;

    /* Write to TIMx CCER */
    TIMx->CCER = tmpccer;
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
