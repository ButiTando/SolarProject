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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define PV_Voltage_Pin GPIO_PIN_0
#define PV_Voltage_GPIO_Port GPIOC
#define TempSensor_Pin GPIO_PIN_1
#define TempSensor_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define PV_Current_Pin GPIO_PIN_4
#define PV_Current_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define S3_Pin GPIO_PIN_7
#define S3_GPIO_Port GPIOA
#define S3_EXTI_IRQn EXTI9_5_IRQn
#define RS_Pin GPIO_PIN_4
#define RS_GPIO_Port GPIOC
#define LuxSensor_Pin GPIO_PIN_5
#define LuxSensor_GPIO_Port GPIOC
#define EN_Pin GPIO_PIN_1
#define EN_GPIO_Port GPIOB
#define Data4_Pin GPIO_PIN_2
#define Data4_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_10
#define D5_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_12
#define D4_GPIO_Port GPIOB
#define D3_Pin GPIO_PIN_13
#define D3_GPIO_Port GPIOB
#define D2_Pin GPIO_PIN_14
#define D2_GPIO_Port GPIOB
#define RNW_Pin GPIO_PIN_15
#define RNW_GPIO_Port GPIOB
#define Data5_Pin GPIO_PIN_8
#define Data5_GPIO_Port GPIOC
#define S1_Pin GPIO_PIN_8
#define S1_GPIO_Port GPIOA
#define S1_EXTI_IRQn EXTI9_5_IRQn
#define S5_Pin GPIO_PIN_9
#define S5_GPIO_Port GPIOA
#define S5_EXTI_IRQn EXTI9_5_IRQn
#define S4_Pin GPIO_PIN_10
#define S4_GPIO_Port GPIOA
#define S4_EXTI_IRQn EXTI15_10_IRQn
#define Data6_Pin GPIO_PIN_11
#define Data6_GPIO_Port GPIOA
#define Data7_Pin GPIO_PIN_12
#define Data7_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define S2_Pin GPIO_PIN_5
#define S2_GPIO_Port GPIOB
#define S2_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

//Student number


// Led flashing frequency. Dependent on timer frequency and count interupt.
#define D2_TICKS (int)100
#define D3_TICKS (int)50
#define D4_TICKS (int)200
#define D5_TICKS (int)100

// Modes. Keep SMODE convention to prevent class with STM modes.
#define SMODE_EN    	(int)1
#define SMODE_SP		(int)2
#define SMODE_CALB  	(int)3
#define SMODE_DATETIME  (int)3

//These are the mode states.
#define IDEAL       (int)0
#define MEASURE     (int)1
#define TX          (int)2

// Menu index
#define DAY 			(int)1
#define MONTH 			(int)2
#define YEAR 			(int)3
#define HOUR 			(int)4
#define MIN 			(int)5
#define SEC 			(int)6
#define SET_DATE_TIME 	(int)7

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
