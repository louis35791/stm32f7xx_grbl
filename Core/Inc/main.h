/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Z_LIMIT_Pin GPIO_PIN_2
#define Z_LIMIT_GPIO_Port GPIOE
#define Z_LIMIT_EXTI_IRQn EXTI2_IRQn
#define CTRL_RESET_Pin GPIO_PIN_3
#define CTRL_RESET_GPIO_Port GPIOE
#define CTRL_RESET_EXTI_IRQn EXTI3_IRQn
#define CTRL_FEED_HOLD_Pin GPIO_PIN_4
#define CTRL_FEED_HOLD_GPIO_Port GPIOE
#define CTRL_FEED_HOLD_EXTI_IRQn EXTI4_IRQn
#define CTRL_CYCLE_START_Pin GPIO_PIN_5
#define CTRL_CYCLE_START_GPIO_Port GPIOE
#define CTRL_CYCLE_START_EXTI_IRQn EXTI9_5_IRQn
#define PROBE_Pin GPIO_PIN_6
#define PROBE_GPIO_Port GPIOE
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define STEP_DISABLE_Pin GPIO_PIN_6
#define STEP_DISABLE_GPIO_Port GPIOF
#define COOLANT_Pin GPIO_PIN_7
#define COOLANT_GPIO_Port GPIOF
#define SPINDLE_ENABLE_Pin GPIO_PIN_8
#define SPINDLE_ENABLE_GPIO_Port GPIOF
#define SPINDLE_DIR_Pin GPIO_PIN_9
#define SPINDLE_DIR_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define Z_PULSE_Pin GPIO_PIN_0
#define Z_PULSE_GPIO_Port GPIOA
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define Y_PULSE_Pin GPIO_PIN_3
#define Y_PULSE_GPIO_Port GPIOA
#define X_DIR_Pin GPIO_PIN_4
#define X_DIR_GPIO_Port GPIOA
#define Z_DIR_Pin GPIO_PIN_5
#define Z_DIR_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define X_PULSE_Pin GPIO_PIN_10
#define X_PULSE_GPIO_Port GPIOB
#define Y_DIR_Pin GPIO_PIN_11
#define Y_DIR_GPIO_Port GPIOB
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define EN_LV_SHIFT_Pin GPIO_PIN_3
#define EN_LV_SHIFT_GPIO_Port GPIOD
#define DEBUG_1_Pin GPIO_PIN_4
#define DEBUG_1_GPIO_Port GPIOD
#define DEBUG_2_Pin GPIO_PIN_5
#define DEBUG_2_GPIO_Port GPIOD
#define DEBUG_3_Pin GPIO_PIN_6
#define DEBUG_3_GPIO_Port GPIOD
#define DEBUG_4_Pin GPIO_PIN_7
#define DEBUG_4_GPIO_Port GPIOD
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define SW0_Pin GPIO_PIN_3
#define SW0_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define X_LIMIT_Pin GPIO_PIN_0
#define X_LIMIT_GPIO_Port GPIOE
#define X_LIMIT_EXTI_IRQn EXTI0_IRQn
#define Y_LIMIT_Pin GPIO_PIN_1
#define Y_LIMIT_GPIO_Port GPIOE
#define Y_LIMIT_EXTI_IRQn EXTI1_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
