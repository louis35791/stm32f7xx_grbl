#include "stm32f7xx_flash_ex.h"

HAL_StatusTypeDef flashPutByte(uint32_t address, uint8_t data)
{
    HAL_FLASH_Unlock();

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS); 

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address, (uint64_t)data) == HAL_OK)
    {
        HAL_FLASH_Lock(); 
        return HAL_OK;
    }
    else
    {
        HAL_FLASH_Lock(); 
        return HAL_FLASH_GetError ();
    }
}

uint8_t flashGetByte(uint32_t address)
{
  return *(__IO uint8_t*)address;
}