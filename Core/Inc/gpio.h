/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#define LED_PIN_ON  LL_GPIO_SetOutputPin(Jump_LED_GPIO_Port, Jump_LED_Pin);
#define LED_PIN_OFF LL_GPIO_ResetOutputPin(Jump_LED_GPIO_Port, Jump_LED_Pin);

//STEP PROGRAMMING
#define MS1_ON LL_GPIO_SetOutputPin(MS1_GPIO_Port, MS1_Pin);
#define MS1_OFF LL_GPIO_ResetOutputPin(MS1_GPIO_Port, MS1_Pin);
#define MS2_ON LL_GPIO_SetOutputPin(MS2_GPIO_Port, MS2_Pin);
#define MS2_OFF LL_GPIO_ResetOutputPin(MS2_GPIO_Port, MS2_Pin);
#define MS3_ON LL_GPIO_SetOutputPin(MS3_GPIO_Port, MS3_Pin);
#define MS3_OFF LL_GPIO_ResetOutputPin(MS3_GPIO_Port, MS3_Pin);

//MOTOR DIRECTION
#define MOTOR_X_RIGHT LL_GPIO_ResetOutputPin(M1_Dir_GPIO_Port, M1_Dir_Pin);
#define MOTOR_X_LEFT  LL_GPIO_SetOutputPin(M1_Dir_GPIO_Port, M1_Dir_Pin);
#define MOTOR_Y_RIGHT LL_GPIO_ResetOutputPin(M2_Dir_GPIO_Port, M2_Dir_Pin);
#define MOTOR_Y_LEFT  LL_GPIO_SetOutputPin(M2_Dir_GPIO_Port, M2_Dir_Pin);

//MOTOR STEP
#define M1_STEP_OFF LL_GPIO_ResetOutputPin(M1_Step_GPIO_Port, M1_Step_Pin);
#define M1_STEP_ON LL_GPIO_SetOutputPin(M1_Step_GPIO_Port, M1_Step_Pin);
#define M2_STEP_OFF LL_GPIO_ResetOutputPin(M2_Step_GPIO_Port, M2_Step_Pin);
#define M2_STEP_ON LL_GPIO_SetOutputPin(M2_Step_GPIO_Port, M2_Step_Pin);

//LINES CONTROL

//ENABLE MOTORS
#define DISABLE_MOTORS LL_GPIO_ResetOutputPin(Enable_GPIO_Port, Enable_Pin);
#define ENABLE_MOTORS  LL_GPIO_SetOutputPin(Enable_GPIO_Port, Enable_Pin);

//RESET DRIVER
#define RESET_OFF_CTRL LL_GPIO_ResetOutputPin(Reset_GPIO_Port, Reset_Pin);
#define RESET_ON_CTRL LL_GPIO_SetOutputPin(Reset_GPIO_Port, Reset_Pin);

//SLEEP MODE
#define SLEEP_OFF_CTRL LL_GPIO_ResetOutputPin(Sleep_GPIO_Port, Sleep_Pin);
#define SLEEP_ON_CTRL LL_GPIO_SetOutputPin(Sleep_GPIO_Port, Sleep_Pin);




/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

