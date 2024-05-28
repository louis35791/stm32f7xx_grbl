#include <string.h>
#include "stm32f7xx_grbl.h"
#include "FreeRTOS.h"
#include "task.h"
#include "step.h"

#define PULSE_FREQ 100000 // Hz

typedef struct TIM_DMA_Parameters
{
    TIM_HandleTypeDef *htim;
    uint8_t TIM_DMA_ID;
    uint8_t TIM_DMA_ID_CC;
    uint8_t TIM_CHANNEL;
    doubleBufferArray_t *preparedBuffer;
    doubleBufferArray_t *preparingBuffer;
    uint16_t lastCounterValue;
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

// declare an array of timer and DMA parameters for each axis
static Axis_TIM_DMA_Parameters_t axisTimerDMAParams[NUM_DIMENSIONS];

// declare an array of buffer for each axis
static AxisBuffer_t axisBuffer[NUM_DIMENSIONS];

// declare an array of general notification
volatile uint32_t generalNotification;

// threshold for determining whether to batch update pulse data.
uint16_t updateBatchThreshold = 0;

/* function prototype */
// void updateBuffer(doubleBufferArray_t *buffer, uint32_t start_index, uint32_t start_value, uint32_t increment, size_t length);
void updateBuffer(doubleBufferArray_t *buffer, uint16_t start_index, uint16_t start_value, uint16_t increment, size_t length);

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
void vStepTask(void *pvParameters)
{
    // initialize step function
    stepInit();

    // compute increment for pulse
    uint16_t increment = COMPUTE_INCREMENT_FOR_PULSE(PULSE_FREQ, APB1_TIMER_CLOCK);

    /**
     * Prepare buffer for each axis in a reverse order
     * to ensure that all the slave timers are ready
     * before the master timer starts.
     */
    for (int8_t i = 0; i >= 0; i--)
    {
        // get timer and DMA parameters of this axis
        TIM_DMA_Parameters_t *timDMAParamsPulse = &axisTimerDMAParams[i].pulse;
        TIM_DMA_Parameters_t *timDMAParamsDir = &axisTimerDMAParams[i].dir;

        // update pulse data
        updateBuffer(&axisBuffer[i].pulse[0], 0, 10, increment, DOUBLE_BUFFER_SIZE);

        // update dir data
        updateBuffer(&axisBuffer[i].dir[0], 0, 10 + (increment / 2), increment, DOUBLE_BUFFER_SIZE);

        // set prepared buffer
        timDMAParamsPulse->preparedBuffer = &axisBuffer[i].pulse[0];
        timDMAParamsDir->preparedBuffer = &axisBuffer[i].dir[0];

        // set preparing buffer
        timDMAParamsPulse->preparingBuffer = &axisBuffer[i].pulse[1];
        timDMAParamsDir->preparingBuffer = &axisBuffer[i].dir[1];

        // set last counter value
        timDMAParamsPulse->lastCounterValue = axisBuffer[i].pulse[0][DOUBLE_BUFFER_SIZE - 1];
        timDMAParamsDir->lastCounterValue = axisBuffer[i].dir[0][DOUBLE_BUFFER_SIZE - 1];

        // set batch update threshold
        if (i == 0)
            updateBatchThreshold = timDMAParamsDir->lastCounterValue;

        // reset timer counter
        __HAL_TIM_SET_COUNTER(timDMAParamsPulse->htim, 0);
        __HAL_TIM_SET_COUNTER(timDMAParamsDir->htim, 0);

        // set length available data
        timDMAParamsPulse->lengthAvailableData = DOUBLE_BUFFER_SIZE;
        timDMAParamsDir->lengthAvailableData = DOUBLE_BUFFER_SIZE;

        // set compare value to the first element of the buffer in timer
        __HAL_TIM_SET_COMPARE(timDMAParamsPulse->htim, timDMAParamsPulse->TIM_CHANNEL, axisBuffer[i].pulse[0][0]);
        __HAL_TIM_SET_COMPARE(timDMAParamsDir->htim, timDMAParamsDir->TIM_CHANNEL, axisBuffer[i].dir[0][0]);

        // start timer output compare mode with DMA
        HAL_TIM_OC_Start_DMA(timDMAParamsPulse->htim, (uint32_t)timDMAParamsPulse->TIM_CHANNEL, ((uint16_t *)(timDMAParamsPulse->preparedBuffer) + 1), timDMAParamsPulse->lengthAvailableData - 1);
        HAL_TIM_OC_Start_DMA(timDMAParamsDir->htim, (uint32_t)timDMAParamsDir->TIM_CHANNEL, ((uint16_t *)(timDMAParamsDir->preparedBuffer) + 1), timDMAParamsDir->lengthAvailableData - 1);
    }

    // Infinite loop
    for (;;)
    {
        /**
         * Batch update pulse data for all axes
         */
        // check if buffer shall be updated
        if ((generalNotification & GENERAL_NOTIFICATION_DATA_READY) == 0)
        {
            for (uint8_t i = 0; i < NUM_DIMENSIONS; i++)
            {
                // get timer and DMA parameters of this axis
                TIM_DMA_Parameters_t *timDMAParamsPulse = &axisTimerDMAParams[i].pulse;
                TIM_DMA_Parameters_t *timDMAParamsDir = &axisTimerDMAParams[i].dir;

                // set length of available data
                timDMAParamsPulse->lengthAvailableData = DOUBLE_BUFFER_SIZE;
                timDMAParamsDir->lengthAvailableData = DOUBLE_BUFFER_SIZE;

                // update pulse data
                updateBuffer(timDMAParamsPulse->preparingBuffer, 0, timDMAParamsPulse->lastCounterValue + increment, increment, timDMAParamsPulse->lengthAvailableData);
                updateBuffer(timDMAParamsDir->preparingBuffer, 0, timDMAParamsDir->lastCounterValue + increment, increment, timDMAParamsDir->lengthAvailableData);

                // set last counter value
                timDMAParamsPulse->lastCounterValue = *(timDMAParamsPulse->preparingBuffer)[timDMAParamsPulse->lengthAvailableData - 1];
                timDMAParamsDir->lastCounterValue = *(timDMAParamsDir->preparingBuffer)[timDMAParamsDir->lengthAvailableData - 1];
            }

            // set notification to data ready
            generalNotification |= GENERAL_NOTIFICATION_DATA_READY;
        }
        // check if there is a notification to resume DMA stream asking from the timer interrupt
        else if (generalNotification & GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE)
        {
            // resume DMA stream with updated buffer and length for each axis
            for (uint8_t i = 0; i < NUM_DIMENSIONS; i++)
            {
                // get timer and DMA parameters of this axis
                TIM_DMA_Parameters_t *timDMAParamsPulse = &axisTimerDMAParams[i].pulse;
                TIM_DMA_Parameters_t *timDMAParamsDir = &axisTimerDMAParams[i].dir;

                // swap buffer
                SWAP_BUFFER_UINT32(timDMAParamsPulse->preparedBuffer, timDMAParamsPulse->preparingBuffer);
                SWAP_BUFFER_UINT32(timDMAParamsDir->preparedBuffer, timDMAParamsDir->preparingBuffer);

                // resume DMA stream with updated buffer and length
                RESUME_DMA_STREAM_WITH_TC(timDMAParamsPulse->htim->hdma[timDMAParamsPulse->TIM_DMA_ID], timDMAParamsPulse->preparedBuffer, timDMAParamsPulse->lengthAvailableData);
                RESUME_DMA_STREAM_WITH_TC(timDMAParamsDir->htim->hdma[timDMAParamsDir->TIM_DMA_ID], timDMAParamsDir->preparedBuffer, timDMAParamsDir->lengthAvailableData);
            }

            // restart counter
            TIM_START_COUNTER(X_AXIS_TIM);

            // reset the notification
            generalNotification &= ~GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE;
        }
    }
}

/**
 * @brief Update double buffer with increment value
 */
void updateBuffer(doubleBufferArray_t *buffer, uint16_t start_index, uint16_t start_value, uint16_t increment, size_t length)
{
    for (uint16_t i = start_index; i < start_index + length; i++)
    {
        (*buffer)[i] = start_value + increment * (i - start_index);
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    // set PD5 to high ===> signal the start of ISR
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);

    // read capture compare register
    uint16_t captureCompareReg = TIM_GET_COMPARE(htim, htim->Channel);

    // check if the capture compare register is equal to the batch update threshold
    if (captureCompareReg != updateBatchThreshold)
    {
        // set PD5 to low ===> signal the end of ISR
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
        return;
    }

    // stop counter
    TIM_STOP_COUNTER(X_AXIS_TIM); // timer  x axis

    // suspend DMA stream and clear interrupt flag
    for (uint8_t i = 0; i < NUM_DIMENSIONS; i++)
    {
        // get timer and DMA parameters of this axis
        TIM_DMA_Parameters_t *timDMAParamsPulse = &axisTimerDMAParams[i].pulse;
        TIM_DMA_Parameters_t *timDMAParamsDir = &axisTimerDMAParams[i].dir;

        // suspend DMA stream
        SUSPEND_DMA_STREAM(timDMAParamsPulse->htim->hdma[timDMAParamsPulse->TIM_DMA_ID]);
        SUSPEND_DMA_STREAM(timDMAParamsDir->htim->hdma[timDMAParamsDir->TIM_DMA_ID]);

        // clear all DMA interrupt flags
        CLEAR_DMA_IT(timDMAParamsPulse->htim->hdma[timDMAParamsPulse->TIM_DMA_ID]);
        CLEAR_DMA_IT(timDMAParamsDir->htim->hdma[timDMAParamsDir->TIM_DMA_ID]);
    }

    // check if the preparing buffer is ready
    if (generalNotification & GENERAL_NOTIFICATION_DATA_READY)
    {
        // set PD4 to high ===> signal the start of updating buffer
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

        for (uint8_t i = 0; i < NUM_DIMENSIONS; i++)
        {
            // get timer and DMA parameters of this axis
            TIM_DMA_Parameters_t *timDMAParamsPulse = &axisTimerDMAParams[i].pulse;
            TIM_DMA_Parameters_t *timDMAParamsDir = &axisTimerDMAParams[i].dir;

            // swap buffer
            SWAP_BUFFER_UINT32(timDMAParamsPulse->preparedBuffer, timDMAParamsPulse->preparingBuffer);
            SWAP_BUFFER_UINT32(timDMAParamsDir->preparedBuffer, timDMAParamsDir->preparingBuffer);

            // resume DMA stream with updated buffer and length
            RESUME_DMA_STREAM_WITH_TC(timDMAParamsPulse->htim->hdma[timDMAParamsPulse->TIM_DMA_ID], timDMAParamsPulse->preparedBuffer, timDMAParamsPulse->lengthAvailableData);
            RESUME_DMA_STREAM_WITH_TC(timDMAParamsDir->htim->hdma[timDMAParamsDir->TIM_DMA_ID], timDMAParamsDir->preparedBuffer, timDMAParamsDir->lengthAvailableData);
        }

        // restart counter
        TIM_START_COUNTER(X_AXIS_TIM);

        // reset the notification
        generalNotification &= ~GENERAL_NOTIFICATION_DATA_READY;

        // set PD4 to low ===> signal the end of updating buffer
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    }
    else
    {
        // notify main task to update pulse data
        generalNotification |= GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE;
    }

    // set PD5 to low ===> signal the end of ISR
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
}
