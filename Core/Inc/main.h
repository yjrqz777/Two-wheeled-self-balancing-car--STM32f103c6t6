/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */



#define ICM20602_SCK_Pin GPIO_PIN_5
#define ICM20602_SCK_GPIO_Port GPIOA

#define ICM20602_MISO_Pin GPIO_PIN_7
#define ICM20602_MISO_GPIO_Port GPIOA

#define ICM20602_MOSI_Pin GPIO_PIN_6
#define ICM20602_MOSI_GPIO_Port GPIOA

#define ICM20602_CS_Pin GPIO_PIN_0
#define ICM20602_CS_GPIO_Port GPIOB
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint8_t xx;
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
//#define ICM20602_SCL_Pin GPIO_PIN_4
//#define ICM20602_SCL_GPIO_Port GPIOA
//#define ICM20602_SDA_Pin GPIO_PIN_5
//#define ICM20602_SDA_GPIO_Port GPIOA
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define key2_Pin GPIO_PIN_14
#define key2_GPIO_Port GPIOC
#define key1_Pin GPIO_PIN_15
#define key1_GPIO_Port GPIOC
#define sck_Pin GPIO_PIN_5
#define sck_GPIO_Port GPIOA
#define mosi_Pin GPIO_PIN_6
#define mosi_GPIO_Port GPIOA
#define miso_Pin GPIO_PIN_7
#define miso_GPIO_Port GPIOA
#define cs_Pin GPIO_PIN_0
#define cs_GPIO_Port GPIOB
#define A2_Pin GPIO_PIN_12
#define A2_GPIO_Port GPIOB
#define A1_Pin GPIO_PIN_13
#define A1_GPIO_Port GPIOB
#define B1_Pin GPIO_PIN_14
#define B1_GPIO_Port GPIOB
#define B2_Pin GPIO_PIN_15
#define B2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
