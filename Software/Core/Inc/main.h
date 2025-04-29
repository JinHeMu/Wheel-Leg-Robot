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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_dwt.h"
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
#define ACC_CS_Pin GPIO_PIN_0
#define ACC_CS_GPIO_Port GPIOC
#define GYRO_CS_Pin GPIO_PIN_3
#define GYRO_CS_GPIO_Port GPIOC
#define DI_Pin GPIO_PIN_0
#define DI_GPIO_Port GPIOA
#define DO_Pin GPIO_PIN_2
#define DO_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_9
#define CS_GPIO_Port GPIOE
#define ACC_INT_Pin GPIO_PIN_10
#define ACC_INT_GPIO_Port GPIOE
#define GYRO_INT_Pin GPIO_PIN_12
#define GYRO_INT_GPIO_Port GPIOE
#define CLK_Pin GPIO_PIN_13
#define CLK_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
typedef float fp32;
typedef double fp64;
#define DI()     HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)

// ?????????????ùI?????PAout(5)??
#define CMD_H()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET)
#define CMD_L()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET)

// CS?????????ùI?????PAout(6)??
#define CS_H()    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET)
#define CS_L()    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET)

// ????????????ùI?????PAout(7)??
#define CLK_H()   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET)
#define CLK_L()   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET)

typedef struct			 				
{
  uint8_t mode;		    /* ?????????? */

  uint8_t btn1;         /* B0:SLCT B3:STRT B4:UP B5:R B6:DOWN B7:L   */

  uint8_t btn2;         /* B0:L2 B1:R2 B2:L1 B3:R1 B4:/\ B5:O B6:X B7:?? */

  uint8_t RJoy_LR;      /* ??????  0x00 = ??    0xff = ??   */

  uint8_t RJoy_UD;      /* ??????  0x00 = ??    0xff = ??   */

  uint8_t LJoy_LR;      /* ??????  0x00 = ??    0xff = ??   */

  uint8_t LJoy_UD;      /* ??????  0x00 = ??    0xff = ??   */
	
	uint8_t last_key;
	
}JOYSTICK_TypeDef;


/*** PS2??????????????? **********/
void AX_PS2_Init(void);  //PS2?????
void AX_PS2_ScanKey(JOYSTICK_TypeDef* JoystickStruct);//PS2??
void ps2_task(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
