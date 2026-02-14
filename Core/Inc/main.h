/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stm32f0xx_ll_crs.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

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
#define MS1_Pin LL_GPIO_PIN_0
#define MS1_GPIO_Port GPIOA
#define MS2_Pin LL_GPIO_PIN_1
#define MS2_GPIO_Port GPIOA
#define Enable_Pin LL_GPIO_PIN_2
#define Enable_GPIO_Port GPIOA
#define MS3_Pin LL_GPIO_PIN_3
#define MS3_GPIO_Port GPIOA
#define Reset_Pin LL_GPIO_PIN_4
#define Reset_GPIO_Port GPIOA
#define M1_Dir_Pin LL_GPIO_PIN_5
#define M1_Dir_GPIO_Port GPIOA
#define M1_Step_Pin LL_GPIO_PIN_6
#define M1_Step_GPIO_Port GPIOA
#define M2_Step_Pin LL_GPIO_PIN_7
#define M2_Step_GPIO_Port GPIOA
#define M2_Dir_Pin LL_GPIO_PIN_1
#define M2_Dir_GPIO_Port GPIOB
#define Jump_LED_Pin LL_GPIO_PIN_13
#define Jump_LED_GPIO_Port GPIOA
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* USER CODE BEGIN Private defines */

#define ON  1
#define OFF 0

#define ENABLE  1
#define DISABLE 0


#define POWER_ON OFF
#define POWER_OFF ON

#define RIGHT OFF
#define LEFT ON

#define MOTOR_X 0
#define MOTOR_Y 1



//https://www.handsontec.com/dataspecs/module/A4988.pdf
typedef enum
{
    STEP_FULL      = 0b00000000,
    STEP_HALF      = 0b00000100,
    STEP_QUARTER   = 0b00000010,
    STEP_EIGHTH    = 0b00000110,
    STEP_SIXTEENTH = 0b00000111

} StepMode_t;

typedef struct relay_ {
      uint8_t status;
      uint8_t recharge;

}relay_st;


typedef struct bit_state_ {
      uint8_t status;
}bit_state_st;

typedef struct step_config_ {
      uint8_t mode;
}step_config_st;

typedef struct dir_status_ {
      uint8_t direction[2];

}dir_status_st;


typedef enum {
    CMD_NONE = 0,
    CMD_VEL,
    CMD_STEP
} cmd_type_t;

typedef struct {
    cmd_type_t type;
    int16_t rpm_x;
    int16_t rpm_y;
    StepMode_t step_mode;
    uint8_t valid;



} cmd_t;



#define RX_BUFFER_SIZE 30
#define QTY_CMD 5
#define CMD_0 "led_off"
#define CMD_1 "led_on"
#define CMD_2 "reset"
#define CMD_3 "nodebug"
#define CMD_4 "update"
#define CMD_DEFINITION {CMD_0,CMD_1,CMD_2,CMD_3,CMD_4} //exemplo, para mem retorna 0, debug retorna 1



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
