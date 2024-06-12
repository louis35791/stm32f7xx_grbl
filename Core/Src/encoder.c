#include <stdlib.h>
#include "stm32f7xx_grbl.h"
#include "stm32f7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "encoder.h"

#define FULL_COUNTER 0xFFFF
#define HALF_COUNTER 0x7FFF
// resolution of degree after decimal point
#define DEGREE_RESOLUTION 100        // 2 decimal points
#define PULSE_PER_REVOLUTION 2000    // 2000 pulses per revolution
#define CALCULATE_RPM_PERIOD_MS 1000 // 1 second = 1000 ms

volatile static int32_t counter32Bit = 0; // store 32-bit counter value
volatile static uint16_t prevCounter = 0; // record previous counter value of 16-bit timer
volatile static int16_t revolution = 0;   // one revolution is 65536, one period of 16-bit timer
volatile uint32_t previousTicks = 0;      // store previous ticks
volatile uint32_t previousDegree = 0;     // store previous degree value
volatile float speedRPM = 0;              // store speed in RPM

void encoderTask(void *pvParameters)
{
    // degree value
    int32_t degree = 0;

    // Infinite loop
    for (;;)
    {

        // Delay for 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void encoderCalculateRPM()
{
    // get current ticks in milliseconds
    uint32_t currentTicks = HAL_GetTick();
    uint32_t diffTicks = currentTicks - previousTicks;

    // if time period is less than 1 second, return
    if (diffTicks < CALCULATE_RPM_PERIOD_MS)
        return;

    // update previous ticks
    previousTicks = currentTicks;

    // get current degree
    int32_t currentDegree = encoderReadDegree();
    int32_t diffDegree = currentDegree - previousDegree;
    // update previous degree
    previousDegree = currentDegree;

    // calculate RPM
    speedRPM = ((float)diffDegree / (360.0 * (float)DEGREE_RESOLUTION)) * (60000.0 / (float)diffTicks);
}

void encoderInterruptHandler(TIM_HandleTypeDef *htim)
{
    uint16_t currentCounter = __HAL_TIM_GET_COUNTER(htim);

    // Detect Overflow / Underflow event to Increase / Decrease revolution counter
    int32_t diff = currentCounter - prevCounter;

    if (diff < 0)
    {
        // overflow occurred
        if (-diff > HALF_COUNTER)
            revolution++;
    }
    else
    {
        // underflow occurred
        if (diff > HALF_COUNTER)
            revolution--;
    }

    counter32Bit = (int32_t)(revolution << 16) + currentCounter;
    prevCounter = currentCounter;

    // calculate RPM
    encoderCalculateRPM();
}

uint32_t encoderGetCounter()
{
    return counter32Bit;
}

void encoderResetCounter(TIM_HandleTypeDef *htim)
{
    counter32Bit = 0;
    prevCounter = 0;
    revolution = 0;

    __HAL_TIM_SET_COUNTER(htim, 0);
}

/**
 * @brief Read the degree value from the encoder
 * @return int32_t degree value x 100
 */
int32_t encoderReadDegree()
{
    return (int32_t)((counter32Bit * 360 * DEGREE_RESOLUTION) / PULSE_PER_REVOLUTION);
}

float encoderReadRPM()
{
    return speedRPM;
}