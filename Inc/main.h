/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
void fmain();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define H_1_NSLEEP_Pin GPIO_PIN_13
#define H_1_NSLEEP_GPIO_Port GPIOC
#define H_1_PHASE_Pin GPIO_PIN_14
#define H_1_PHASE_GPIO_Port GPIOC
#define H_1_MODE_Pin GPIO_PIN_15
#define H_1_MODE_GPIO_Port GPIOC
#define H_2_FAULT_Pin GPIO_PIN_0
#define H_2_FAULT_GPIO_Port GPIOC
#define H_1_FAULT_Pin GPIO_PIN_1
#define H_1_FAULT_GPIO_Port GPIOC
#define H1_VPROPI_Pin GPIO_PIN_3
#define H1_VPROPI_GPIO_Port GPIOC
#define H_3_FAULT_Pin GPIO_PIN_5
#define H_3_FAULT_GPIO_Port GPIOC
#define IMU_INT_1_Pin GPIO_PIN_2
#define IMU_INT_1_GPIO_Port GPIOB
#define H_3_NSLEEP_Pin GPIO_PIN_12
#define H_3_NSLEEP_GPIO_Port GPIOB
#define H_3_PHASE_Pin GPIO_PIN_13
#define H_3_PHASE_GPIO_Port GPIOB
#define H_4_FAULT_Pin GPIO_PIN_14
#define H_4_FAULT_GPIO_Port GPIOB
#define IMU_INT_2_Pin GPIO_PIN_6
#define IMU_INT_2_GPIO_Port GPIOC
#define H_3_MODE_Pin GPIO_PIN_8
#define H_3_MODE_GPIO_Port GPIOC
#define H_4_MODE_Pin GPIO_PIN_9
#define H_4_MODE_GPIO_Port GPIOC
#define H_4_NSLEEP_Pin GPIO_PIN_12
#define H_4_NSLEEP_GPIO_Port GPIOA
#define H_4_PHASE_Pin GPIO_PIN_11
#define H_4_PHASE_GPIO_Port GPIOC
#define H_2_MODE_Pin GPIO_PIN_12
#define H_2_MODE_GPIO_Port GPIOC
#define H_2_NSLEEP_Pin GPIO_PIN_4
#define H_2_NSLEEP_GPIO_Port GPIOB
#define H_2_PHASE_Pin GPIO_PIN_5
#define H_2_PHASE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
