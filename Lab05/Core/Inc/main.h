/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define J1_LED2_Pin GPIO_PIN_13
#define J1_LED2_GPIO_Port GPIOC
#define StartRace_Pin GPIO_PIN_0
#define StartRace_GPIO_Port GPIOC
#define StartRace_EXTI_IRQn EXTI0_IRQn
#define J1_WIN_Pin GPIO_PIN_1
#define J1_WIN_GPIO_Port GPIOC
#define J1_LED3_Pin GPIO_PIN_2
#define J1_LED3_GPIO_Port GPIOC
#define J1_LED4_Pin GPIO_PIN_3
#define J1_LED4_GPIO_Port GPIOC
#define J1_LED5_Pin GPIO_PIN_0
#define J1_LED5_GPIO_Port GPIOA
#define J1_LED6_Pin GPIO_PIN_1
#define J1_LED6_GPIO_Port GPIOA
#define J1_LED7_Pin GPIO_PIN_4
#define J1_LED7_GPIO_Port GPIOA
#define J2_LED1_Pin GPIO_PIN_6
#define J2_LED1_GPIO_Port GPIOA
#define J2_LED2_Pin GPIO_PIN_7
#define J2_LED2_GPIO_Port GPIOA
#define LED_AMARILLO_Pin GPIO_PIN_4
#define LED_AMARILLO_GPIO_Port GPIOC
#define J1_LED8_Pin GPIO_PIN_0
#define J1_LED8_GPIO_Port GPIOB
#define J2_LED7_Pin GPIO_PIN_10
#define J2_LED7_GPIO_Port GPIOB
#define J2_LED4_Pin GPIO_PIN_7
#define J2_LED4_GPIO_Port GPIOC
#define PJ2_Pin GPIO_PIN_8
#define PJ2_GPIO_Port GPIOC
#define PJ2_EXTI_IRQn EXTI9_5_IRQn
#define J2_LED6_Pin GPIO_PIN_8
#define J2_LED6_GPIO_Port GPIOA
#define J2_LED5_Pin GPIO_PIN_9
#define J2_LED5_GPIO_Port GPIOA
#define LED_ROJO_Pin GPIO_PIN_10
#define LED_ROJO_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define PJ1_Pin GPIO_PIN_10
#define PJ1_GPIO_Port GPIOC
#define PJ1_EXTI_IRQn EXTI15_10_IRQn
#define J1_LED1_Pin GPIO_PIN_12
#define J1_LED1_GPIO_Port GPIOC
#define LED_VERDE_Pin GPIO_PIN_3
#define LED_VERDE_GPIO_Port GPIOB
#define J2_LED8_Pin GPIO_PIN_4
#define J2_LED8_GPIO_Port GPIOB
#define J2_WIN_Pin GPIO_PIN_5
#define J2_WIN_GPIO_Port GPIOB
#define J2_LED3_Pin GPIO_PIN_6
#define J2_LED3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
