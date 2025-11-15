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
#include "stm32h7xx_hal.h"

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
#define CODEC_NRST_Pin GPIO_PIN_13
#define CODEC_NRST_GPIO_Port GPIOC
#define POT_7_Pin GPIO_PIN_0
#define POT_7_GPIO_Port GPIOC
#define POT_6_Pin GPIO_PIN_1
#define POT_6_GPIO_Port GPIOC
#define PLED_7_Pin GPIO_PIN_2
#define PLED_7_GPIO_Port GPIOC
#define PLED_6_Pin GPIO_PIN_3
#define PLED_6_GPIO_Port GPIOC
#define PLED_5_Pin GPIO_PIN_0
#define PLED_5_GPIO_Port GPIOA
#define PLED_4_Pin GPIO_PIN_1
#define PLED_4_GPIO_Port GPIOA
#define POT_5_Pin GPIO_PIN_2
#define POT_5_GPIO_Port GPIOA
#define POT_4_Pin GPIO_PIN_3
#define POT_4_GPIO_Port GPIOA
#define POT_3_Pin GPIO_PIN_4
#define POT_3_GPIO_Port GPIOA
#define PWLED_3_Pin GPIO_PIN_5
#define PWLED_3_GPIO_Port GPIOA
#define EXP_PED_Pin GPIO_PIN_6
#define EXP_PED_GPIO_Port GPIOA
#define PWLED_2_Pin GPIO_PIN_7
#define PWLED_2_GPIO_Port GPIOA
#define POT_0_Pin GPIO_PIN_4
#define POT_0_GPIO_Port GPIOC
#define POT_2_Pin GPIO_PIN_5
#define POT_2_GPIO_Port GPIOC
#define POT_1_Pin GPIO_PIN_0
#define POT_1_GPIO_Port GPIOB
#define PLED_0_Pin GPIO_PIN_1
#define PLED_0_GPIO_Port GPIOB
#define PLED_1_Pin GPIO_PIN_2
#define PLED_1_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_10
#define LED_GPIO_Port GPIOD
#define EN_BT_Pin GPIO_PIN_8
#define EN_BT_GPIO_Port GPIOC
#define ENC_A_Pin GPIO_PIN_8
#define ENC_A_GPIO_Port GPIOA
#define ENC_B_Pin GPIO_PIN_9
#define ENC_B_GPIO_Port GPIOA
#define SPI3_DC_Pin GPIO_PIN_0
#define SPI3_DC_GPIO_Port GPIOD
#define SPI3_RES_Pin GPIO_PIN_1
#define SPI3_RES_GPIO_Port GPIOD
#define FT_2_Pin GPIO_PIN_9
#define FT_2_GPIO_Port GPIOB
#define FT_1_Pin GPIO_PIN_0
#define FT_1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
