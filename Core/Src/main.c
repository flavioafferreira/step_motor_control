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
 * USART SERIAL SPEED 1MBPS
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

relay_st relay;
relay_st relay_on_off(uint8_t control);
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


void exec_cmd(uint8_t cmd){
       switch(cmd){

       case 0:
    	   printf("CMD MEM\n\r");
    	   break;
       case 1:
    	   break;
       case 2:
    	   break;
       case 3:
    	   break;
       case 4:
    	   break;
       default:

       }

}


//redirect the printf to usart1
int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) {
        while (!LL_USART_IsActiveFlag_TXE(USART1));  // Wait for the buffer to be empty
        LL_USART_TransmitData8(USART1, ptr[i]);  // Send a character
    }
    while (!LL_USART_IsActiveFlag_TC(USART1));  // Wait for the transmission to finish
    return len;
}

void print_float(float number) {
    int32_t mantissa = (int32_t)number;  // Integer part
    float intermediate = number - mantissa;
    int32_t expoente = (int32_t)(intermediate * 100000); // Decimal part (5 digits)

    // Ensure the exponent is positive
    if (expoente < 0) expoente *= -1;

    // Correct printing ensuring 5 digits in the decimal part
    printf("%ld.%05ld", mantissa, expoente);
}

void print_float_1dec(float number) {
    int32_t mantissa = (int32_t)number;  // Integer part
    float intermediate = number - mantissa;
    int32_t expoente = (int32_t)(intermediate * 100000); // Decimal part (5 digits)

    // Ensure the exponent is positive
    if (expoente < 0) expoente *= -1;

    while (expoente >= 10) {
    	expoente /= 10;
        }
    // Correct printing ensuring 1 digit
    printf("%ld.%ld", mantissa, expoente);
}


void sort(float* data, int n) {
    for (int i = 0; i < n - 1; ++i) {

        for (int j = 0; j < n - i - 1; ++j) {
            if (data[j] > data[j + 1]) {
                float temp = data[j];
                data[j] = data[j + 1];
                data[j + 1] = temp;
            }
        }
    }
}

float trimmed_mean(float* data, int n, float trim_percentage) {
    int trim_count = (int)(n * trim_percentage / 100);
    sort(data, n);
    float sum = 0.0;
    for (int i = trim_count; i < n - trim_count; ++i) {
        sum += data[i];
    }
    int remaining_elements = n - 2 * trim_count;
    float trimmed_mean_value = sum / remaining_elements;
    return trimmed_mean_value;
}

void cmd_analise_task(void){
	int result;
        uint8_t i=0;
		while (i<QTY_CMD){
		 result = strcmp(str_cmd[i], rx_buffer);
		 if (!result)exec_cmd(i);
	     i++;
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
  /* USER CODE BEGIN 2 */

  printf("StepMotor OK\n\r");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  LL_GPIO_SetOutputPin(M1_Step_GPIO_Port, M1_Step_Pin);
	  LL_GPIO_ResetOutputPin(M1_Dir_GPIO_Port, M1_Dir_Pin);
	  LL_mDelay(50);
	  LL_GPIO_SetOutputPin(M1_Dir_GPIO_Port, M1_Dir_Pin);
	  LL_GPIO_ResetOutputPin(M1_Step_GPIO_Port, M1_Step_Pin);
	  LL_mDelay(50);

	  printf("Loop\n\r");
	  LL_mDelay(500);



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
