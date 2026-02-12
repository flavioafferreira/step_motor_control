/*
 * special.c
 *
 *  Created on: Dec 2, 2025
 *      Author: f.ferreira
 */

#include "special.h"
#include "stdio.h"
#include "math.h"
#include "main.h"
#include "gpio.h"


relay_st relay_on_off(uint8_t control){
   relay_st relay_ctrl;
   if (control==POWER_ON){LED_PIN_ON;}
   if (control==POWER_OFF){LED_PIN_OFF;} //HEATER_PIN_ON;
   relay_ctrl.status=control;
   return relay_ctrl;
}


//redirect the printf to usart1
int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) {
        while (!LL_USART_IsActiveFlag_TXE(USART1));  // Espera o buffer ficar vazio
        LL_USART_TransmitData8(USART1, ptr[i]);  // Envia um caractere
    }
    while (!LL_USART_IsActiveFlag_TC(USART1));  // Espera a transmissão terminar
    return len;
}



void print_float(float number) {
    int32_t mantissa = (int32_t)number;  // Parte inteira
    float intermediate = number - mantissa;
    int32_t expoente = (int32_t)(intermediate * 100000); // Parte decimal (5 casas)

    // Garantir que expoente seja positivo
    if (expoente < 0) expoente *= -1;

    // Impressão correta garantindo 5 dígitos no decimal
    printf("%ld.%05ld", mantissa, expoente);
}

void print_float_1dec(float number) {
    int32_t mantissa = (int32_t)number;  // Parte inteira
    float intermediate = number - mantissa;
    int32_t expoente = (int32_t)(intermediate * 100000); // Parte decimal (5 casas)

    // Garantir que expoente seja positivo
    if (expoente < 0) expoente *= -1;

    while (expoente >= 10) {
    	expoente /= 10;
        }
    // Impressão correta garantindo com 1 dígito
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



//DRIVER STEP PROGRAMMING

step_config_st step_configuration(uint8_t value){
	//steps from 0 up to 15
	step_config_st step_config;
   if (value==STEP_FULL)     {MS1_OFF; MS2_OFF; MS3_OFF};
   if (value==STEP_HALF)     {MS1_ON ; MS2_OFF; MS3_OFF};
   if (value==STEP_QUARTER)  {MS1_OFF; MS2_ON;  MS3_OFF};
   if (value==STEP_EIGHTH)   {MS1_ON ; MS2_ON;  MS3_OFF};
   if (value==STEP_SIXTEENTH){MS1_ON ; MS2_ON;  MS3_ON };

   step_config.step=value;
   return step_config;
}


dir_status_st motor_dir(uint8_t motor_number, uint8_t direction){
	dir_status_st internal_dir;

	if (motor_number==MOTOR_X && direction==RIGHT){MOTOR_X_RIGHT}
	if (motor_number==MOTOR_X && direction==LEFT){MOTOR_X_LEFT}

	if (motor_number==MOTOR_Y && direction==RIGHT){MOTOR_Y_RIGHT}
	if (motor_number==MOTOR_Y && direction==LEFT){MOTOR_Y_LEFT}

	internal_dir.motor_number=motor_number;
	internal_dir.direction=direction;
	return internal_dir;
}

void reset_driver(void){
	RESET_ON_CTRL;
	LL_mDelay(50);
	RESET_OFF_CTRL;
}

bit_state_st sleep_driver(uint8_t status){
	bit_state_st internal;

	if(status)SLEEP_ON_CTRL
	else SLEEP_OFF_CTRL

	internal.status=status;
	return internal;
}

bit_state_st enable_motors(uint8_t status){
	bit_state_st internal;

	if(status)ENABLE_MOTORS
	else DISABLE_MOTORS

	internal.status=status;
	return internal;


}




