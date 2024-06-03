#include <string.h>
#include "stm32f7xx_hal.h"
#include "stm32f7xx_grbl.h"
#include "FreeRTOS.h"
#include "task.h"
#include "step.h"

#define PULSE_FREQ 300000 // Hz
#define HALF_PERIOD 0x7FFF // 0xFFFF / 2

typedef struct TIM_DMA_Parameters
{
    TIM_HandleTypeDef *htim;
    uint8_t TIM_DMA_ID;
    uint8_t TIM_DMA_ID_CC;
    uint8_t TIM_CHANNEL;
    doubleBufferArray_t *preparedBuffer;
    doubleBufferArray_t *preparingBuffer;
    uint32_t lastCounterValue;
    uint32_t secondLastCounterValue;
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
volatile uint32_t updateBatchThreshold = 0;
volatile uint32_t nextUpdateBatchThreshold = 0;

// increment for pulse
volatile uint16_t increment = 0;

// counter overruns the batch update threshold
volatile uint8_t overrun = 0;

// capture compare register value
volatile uint16_t captureCompareReg = 0;

/* function prototype */
void updateBuffer(doubleBufferArray_t *buffer, uint32_t start_index, uint32_t start_value, uint32_t increment, size_t length);
static void TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma);

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

    // set PD3 to high to enable level shifter
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);

    // compute increment for pulse
    // increment = COMPUTE_INCREMENT_FOR_PULSE(1000, APB1_TIMER_CLOCK);
    increment = COMPUTE_INCREMENT_FOR_PULSE(PULSE_FREQ, APB1_TIMER_CLOCK);

    /**
     * Prepare buffer for each axis in a reverse order
     * to ensure that all the slave timers are ready
     * before the master timer starts.
     */
    for (int8_t i = NUM_DIMENSIONS - 1; i >= 0; i--)
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
        TIM_OC_Start_DMA(timDMAParamsPulse->htim, (uint32_t)timDMAParamsPulse->TIM_CHANNEL, ((uint32_t *)(timDMAParamsPulse->preparedBuffer) + 1), timDMAParamsPulse->lengthAvailableData - 1);
        TIM_OC_Start_DMA(timDMAParamsDir->htim, (uint32_t)timDMAParamsDir->TIM_CHANNEL, ((uint32_t *)(timDMAParamsDir->preparedBuffer) + 1), timDMAParamsDir->lengthAvailableData - 1);
        __HAL_TIM_ENABLE(timDMAParamsPulse->htim);
    }

    // compute increment for pulse
    increment = COMPUTE_INCREMENT_FOR_PULSE(PULSE_FREQ, APB1_TIMER_CLOCK);

    // Infinite loop
    for (;;)
    {
        /**
         * Batch update pulse data for all axes
         */
        // check if buffer shall be updated
        if ((generalNotification & GENERAL_NOTIFICATION_DATA_READY) == 0)
        {
            // set PD4 to high ===> signal the start of updating buffer
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

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
                timDMAParamsPulse->lastCounterValue = (*timDMAParamsPulse->preparingBuffer)[timDMAParamsPulse->lengthAvailableData - 1];
                timDMAParamsDir->lastCounterValue = (*timDMAParamsDir->preparingBuffer)[timDMAParamsDir->lengthAvailableData - 1];

                // set batch update threshold
                if (i == 0)
                    nextUpdateBatchThreshold = timDMAParamsDir->lastCounterValue;
            }

            // set notification to data ready
            generalNotification |= GENERAL_NOTIFICATION_DATA_READY;

            // set PD4 to low ===> signal the end of updating buffer
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
        }
        // check if there is a notification to resume DMA stream asking from the timer interrupt
        else if (generalNotification & GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE)
        {
            // set PD5 to high ===> signal the start of resuming DMA stream from task
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);

            // resume DMA stream with updated buffer and length for each axis
            for (uint8_t i = 0; i < NUM_DIMENSIONS; i++)
            {
                // get timer and DMA parameters of this axis
                TIM_DMA_Parameters_t *timDMAParamsPulse = &axisTimerDMAParams[i].pulse;
                TIM_DMA_Parameters_t *timDMAParamsDir = &axisTimerDMAParams[i].dir;

                // swap buffer
                SWAP_BUFFER_UINT32(timDMAParamsPulse->preparedBuffer, timDMAParamsPulse->preparingBuffer);
                SWAP_BUFFER_UINT32(timDMAParamsDir->preparedBuffer, timDMAParamsDir->preparingBuffer);

                // check if counter matches compare value
                if (overrun)
                {
                    // set counter to capture compare register + 1
                    __HAL_TIM_SET_COUNTER(timDMAParamsPulse->htim, captureCompareReg + 1);
                }

                // set compare value to the first element of the buffer in timer
                // __HAL_TIM_SET_COMPARE(timDMAParamsPulse->htim, timDMAParamsPulse->TIM_CHANNEL, (*timDMAParamsPulse->preparedBuffer)[0]);

                // resume DMA stream with updated buffer and length
                RESUME_DMA_STREAM_WITH_TC(timDMAParamsPulse->htim->hdma[timDMAParamsPulse->TIM_DMA_ID], timDMAParamsPulse->preparedBuffer, timDMAParamsPulse->lengthAvailableData - 1);
                RESUME_DMA_STREAM_WITH_TC(timDMAParamsDir->htim->hdma[timDMAParamsDir->TIM_DMA_ID], timDMAParamsDir->preparedBuffer, timDMAParamsDir->lengthAvailableData);
            }
            // set batch update threshold with the next batch update threshold
            updateBatchThreshold = nextUpdateBatchThreshold;

            // restart counter
            TIM_START_COUNTER(X_AXIS_TIM);

            // reset the notification
            generalNotification &= ~GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE;

            // set PD5 to low ===> signal the end of resuming DMA stream from task
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
        }
    }
}

/**
 * @brief Update double buffer with increment value
 */
void updateBuffer(doubleBufferArray_t *buffer, uint32_t start_index, uint32_t start_value, uint32_t increment, size_t length)
{
    for (uint16_t i = start_index; i < start_index + length; i++)
    {
        (*buffer)[i] = (uint32_t)((uint16_t)(start_value + increment * (i - start_index)));
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
HAL_StatusTypeDef TIM_OC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, const uint32_t *pData,
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
    TIM_STOP_COUNTER(X_AXIS_TIM); // timer x axis

    // set PD6 to high ===> signal the start of ISR
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);

    // read capture compare register
    captureCompareReg = TIM_GET_COMPARE(htim, htim->Channel);

    // check if the capture compare register is equal to the batch update threshold
    if (captureCompareReg != updateBatchThreshold)
    {
        // restart counter
        TIM_START_COUNTER(X_AXIS_TIM);

        // set PD5 to low ===> signal the end of ISR
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
        return;
    }

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

    // check if counter overruns the batch update threshold
    uint32_t counter = __HAL_TIM_GET_COUNTER(X_AXIS_TIM_HANDLE);
    int32_t diff =  counter - updateBatchThreshold;
    if((diff > 0) && (diff < HALF_PERIOD)) overrun = 1;
    else if((diff < 0) && (diff < -HALF_PERIOD)) overrun = 1;
    else overrun = 0;

    // check if the preparing buffer is ready
    if (generalNotification & GENERAL_NOTIFICATION_DATA_READY)
    {
        // set PD7 to high ===> signal the start of updating DMA buffer
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

        for (uint8_t i = 0; i < NUM_DIMENSIONS; i++)
        {
            // get timer and DMA parameters of this axis
            TIM_DMA_Parameters_t *timDMAParamsPulse = &axisTimerDMAParams[i].pulse;
            TIM_DMA_Parameters_t *timDMAParamsDir = &axisTimerDMAParams[i].dir;

            // swap buffer
            SWAP_BUFFER_UINT32(timDMAParamsPulse->preparedBuffer, timDMAParamsPulse->preparingBuffer);
            SWAP_BUFFER_UINT32(timDMAParamsDir->preparedBuffer, timDMAParamsDir->preparingBuffer);

            // check if counter matches compare value
            if (overrun)
            {
                // set counter to capture compare register + 1
                __HAL_TIM_SET_COUNTER(timDMAParamsPulse->htim, captureCompareReg + 1);
            }

            // set compare value to the first element of the buffer in timer
            // __HAL_TIM_SET_COMPARE(timDMAParamsPulse->htim, timDMAParamsPulse->TIM_CHANNEL, (*timDMAParamsPulse->preparedBuffer)[0]);

            // resume DMA stream with updated buffer and length
            // RESUME_DMA_STREAM_WITH_TC(timDMAParamsPulse->htim->hdma[timDMAParamsPulse->TIM_DMA_ID], ((uint32_t *)(timDMAParamsPulse->preparedBuffer) + 1), timDMAParamsPulse->lengthAvailableData - 1);
            RESUME_DMA_STREAM_WITH_TC(timDMAParamsPulse->htim->hdma[timDMAParamsPulse->TIM_DMA_ID], timDMAParamsPulse->preparedBuffer, timDMAParamsPulse->lengthAvailableData);
            RESUME_DMA_STREAM_WITH_TC(timDMAParamsDir->htim->hdma[timDMAParamsDir->TIM_DMA_ID], timDMAParamsDir->preparedBuffer, timDMAParamsDir->lengthAvailableData);
        }
        // set batch update threshold with the next batch update threshold
        updateBatchThreshold = nextUpdateBatchThreshold;

        // restart counter
        TIM_START_COUNTER(X_AXIS_TIM);

        // reset the notification
        generalNotification &= ~GENERAL_NOTIFICATION_DATA_READY;

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
