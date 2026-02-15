/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/*
 * USART SERIAL SPEED 115kbps
 *
 *
 *
 *
 *
 */



/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "string.h"
#include "special.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

relay_st relay;
bit_state_st enable_driver;
step_config_st step_config;  //DONE
dir_status_st motor_direction; //DONE

extern volatile uint8_t rx_line_ready;


char *str_cmd[QTY_CMD] = CMD_DEFINITION;
extern char rx_buffer[RX_BUFFER_SIZE];


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void cmd_analise_task(void){

	if (rx_line_ready) {
	    rx_line_ready = 0;
	    cmd_t cmd = parse_line((char*)rx_buffer);
	    if (cmd.valid){
	        apply_cmd(&cmd);
		    }
		}


}


void timer16_init(void){


	//for microstep 1/16 use 500us = 37.5rpm
	//for fullstep 1/1 use   8ms = 37.5rpm, maximum 1500us for 200rpm


	LL_TIM_SetCounter(TIM16, 0);
	LL_TIM_SetAutoReload(TIM16,9 ); //9=10us ate 40.01ms
	LL_TIM_OC_SetCompareCH1(TIM16, 1);

	LL_TIM_EnableAllOutputs(TIM16);               // importante no TIM16
	LL_TIM_CC_EnableChannel(TIM16, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableCounter(TIM16);
	LL_TIM_GenerateEvent_UPDATE(TIM16);

}


void timer17_init(void){


	//for microstep 1/16 use 500us = 37.5rpm
	//for fullstep 1/1 use   8ms = 37.5rpm, maximum 1500us for 200rpm


	LL_TIM_SetCounter(TIM17, 0);
	LL_TIM_SetAutoReload(TIM17,9 ); //9=10us ate 40.01ms
	LL_TIM_OC_SetCompareCH1(TIM17, 1);

	LL_TIM_EnableAllOutputs(TIM17);               // importante no TIM16
	LL_TIM_CC_EnableChannel(TIM17, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableCounter(TIM17);
	LL_TIM_GenerateEvent_UPDATE(TIM17);

}


void test_motors(void){

	  uint8_t i=0;
	  uint8_t lado_giro=LEFT;

	  while(1){
	  i++;
	  if (i>=200){
		  if(lado_giro==LEFT){
			  	  	  	  	   motor_direction=motor_dir(MOTOR_X, RIGHT);
		                       motor_direction=motor_dir(MOTOR_Y, RIGHT);
		                       lado_giro=RIGHT;
		                       Motor_SetRPM(MOTOR_X,200.0);
		                       Motor_SetRPM(MOTOR_Y,200.0);

		    }else  if(lado_giro==RIGHT){
		    					motor_direction=motor_dir(MOTOR_X, LEFT);
		                        motor_direction=motor_dir(MOTOR_Y, LEFT);
		                        lado_giro=LEFT;
		                        Motor_SetRPM(MOTOR_X,200.0);
		                        Motor_SetRPM(MOTOR_Y,200.0);
		           }
		  i=0;
	  }

	   LL_mDelay(2);
	  }


}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

  timer16_init();
  timer17_init();

  printf("\n\r\n\r<Ready>\n\r");

  printf("COMMANDS\n\r");
  printf("S 1  <cr> = STEP_FULL\n\r");
  printf("S 16 <cr> = STEP_SIXTEENTH\n\r");
  printf("V 100 -120 <cr> = MOTOR X RIGHT DIRECTION 100rpm and MOTOR Y LEFT DIRECTION 120rpm\n\r");

  relay=relay_on_off(POWER_OFF);


  step_config=step_configuration(STEP_FULL);
  reset_driver();                       //PA13

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  LL_mDelay(2);
	  cmd_analise_task(); //verify input commands
	  Motor_RampTask_10ms();
	  //test_motors();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_5);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(40000000);
  LL_SetSystemCoreClock(40000000);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
