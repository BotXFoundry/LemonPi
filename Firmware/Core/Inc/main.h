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
extern void Handle_Battery_Data();
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI1_CS_Pin GPIO_PIN_2
#define SPI1_CS_GPIO_Port GPIOE
#define SPI1_MOSI_Pin GPIO_PIN_3
#define SPI1_MOSI_GPIO_Port GPIOE
#define SPI1_MISO_Pin GPIO_PIN_4
#define SPI1_MISO_GPIO_Port GPIOE
#define SPI2_SCK_Pin GPIO_PIN_5
#define SPI2_SCK_GPIO_Port GPIOE
#define SPI2_CS_Pin GPIO_PIN_6
#define SPI2_CS_GPIO_Port GPIOE
#define GPIO_9_Pin GPIO_PIN_13
#define GPIO_9_GPIO_Port GPIOC
#define Sharp_IR1_Pin GPIO_PIN_0
#define Sharp_IR1_GPIO_Port GPIOC
#define Sharp_IR2_Pin GPIO_PIN_2
#define Sharp_IR2_GPIO_Port GPIOC
#define Sharp_IR3_Pin GPIO_PIN_3
#define Sharp_IR3_GPIO_Port GPIOC
#define Sharp_IR4_Pin GPIO_PIN_0
#define Sharp_IR4_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_7
#define LED3_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_8
#define LED2_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOE
#define SPI3_MISO_Pin GPIO_PIN_10
#define SPI3_MISO_GPIO_Port GPIOE
#define SPI3_MOSI_Pin GPIO_PIN_11
#define SPI3_MOSI_GPIO_Port GPIOE
#define SPI3_CS_Pin GPIO_PIN_12
#define SPI3_CS_GPIO_Port GPIOE
#define SPI3_SCK_Pin GPIO_PIN_13
#define SPI3_SCK_GPIO_Port GPIOE
#define SPI2_MISO_Pin GPIO_PIN_14
#define SPI2_MISO_GPIO_Port GPIOE
#define SPI2_MOSI_Pin GPIO_PIN_15
#define SPI2_MOSI_GPIO_Port GPIOE
#define RS485_EN2_Pin GPIO_PIN_10
#define RS485_EN2_GPIO_Port GPIOD
#define RS485_EN1_Pin GPIO_PIN_11
#define RS485_EN1_GPIO_Port GPIOD
#define G_PD12_Pin GPIO_PIN_12
#define G_PD12_GPIO_Port GPIOD
#define G_PD13_Pin GPIO_PIN_13
#define G_PD13_GPIO_Port GPIOD
#define G_PD14_Pin GPIO_PIN_14
#define G_PD14_GPIO_Port GPIOD
#define G_PD15_Pin GPIO_PIN_15
#define G_PD15_GPIO_Port GPIOD
#define RS485_TX2_Pin GPIO_PIN_6
#define RS485_TX2_GPIO_Port GPIOC
#define RS485_RX2_Pin GPIO_PIN_7
#define RS485_RX2_GPIO_Port GPIOC
#define CRASHPROOF_1_Pin GPIO_PIN_8
#define CRASHPROOF_1_GPIO_Port GPIOC
#define RS485_TX1_Pin GPIO_PIN_12
#define RS485_TX1_GPIO_Port GPIOC
#define RS485_RX1_Pin GPIO_PIN_2
#define RS485_RX1_GPIO_Port GPIOD
#define GPIO_8_Pin GPIO_PIN_3
#define GPIO_8_GPIO_Port GPIOD
#define GPIO_7_Pin GPIO_PIN_4
#define GPIO_7_GPIO_Port GPIOD
#define CRASHPROOF_2_Pin GPIO_PIN_7
#define CRASHPROOF_2_GPIO_Port GPIOD
#define SPI1_SCK_Pin GPIO_PIN_0
#define SPI1_SCK_GPIO_Port GPIOE
#define ETH_RESET_Pin GPIO_PIN_1
#define ETH_RESET_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
