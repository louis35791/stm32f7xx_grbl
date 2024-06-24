#include "stm32f7xx_grbl.h"
#include "utils.h"

TIM_HandleTypeDef* htimUs; // Timer for microsecond delay

void utilsStartUsTimer(TIM_HandleTypeDef *htim)
{
    htimUs = htim;
    HAL_TIM_Base_Start(htimUs);
}

void utilsDelayUs(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(htimUs, 0);
    while (__HAL_TIM_GET_COUNTER(htimUs) < us);
}