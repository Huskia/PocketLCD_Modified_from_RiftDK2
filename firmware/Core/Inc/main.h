/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32l1xx_hal.h"

#include "stm32l1xx_ll_i2c.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_exti.h"
#include "stm32l1xx_ll_cortex.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_dma.h"
#include "stm32l1xx_ll_tim.h"
#include "stm32l1xx_ll_usart.h"
#include "stm32l1xx_ll_gpio.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TC_INT_Pin LL_GPIO_PIN_13
#define TC_INT_GPIO_Port GPIOC
#define TC_INT_EXTI_IRQn EXTI15_10_IRQn
#define TC_STDBY_Pin LL_GPIO_PIN_15
#define TC_STDBY_GPIO_Port GPIOC
#define LED_B_Pin LL_GPIO_PIN_0
#define LED_B_GPIO_Port GPIOC
#define LED_Y_Pin LL_GPIO_PIN_1
#define LED_Y_GPIO_Port GPIOC
#define OLED_RST_Pin LL_GPIO_PIN_3
#define OLED_RST_GPIO_Port GPIOC
#define OLED_ERR_Pin LL_GPIO_PIN_0
#define OLED_ERR_GPIO_Port GPIOA
#define TC_PWR_Pin LL_GPIO_PIN_0
#define TC_PWR_GPIO_Port GPIOB
#define TC_RST_Pin LL_GPIO_PIN_9
#define TC_RST_GPIO_Port GPIOC
#define KEY_M_Pin LL_GPIO_PIN_2
#define KEY_M_GPIO_Port GPIOD
#define KEY_P_Pin LL_GPIO_PIN_3
#define KEY_P_GPIO_Port GPIOB
#define PWR_KEY_Pin LL_GPIO_PIN_4
#define PWR_KEY_GPIO_Port GPIOB
#define I2C1_INT_Pin LL_GPIO_PIN_5
#define I2C1_INT_GPIO_Port GPIOB
#define I2C1_INT_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
