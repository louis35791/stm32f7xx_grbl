
#ifndef __STM32F7xx_FLASH_EX_H
#define __STM32F7xx_FLASH_EX_H

#include "stm32f7xx_hal.h"

/**
 * Define Function Prototypes
 */
HAL_StatusTypeDef flashPutByte(uint32_t address, uint8_t data);
uint8_t flashGetByte(uint32_t address);

#endif // __STM32F7xx_FLASH_EX_H