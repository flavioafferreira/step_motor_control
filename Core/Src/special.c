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

#include <ctype.h>
#include <stdlib.h>


extern bit_state_st enable_driver;
extern step_config_st step_config;
extern dir_status_st motor_direction;


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


uint8_t rpm_divider(step_config_st step_config){
	switch (step_config.mode)
    {
        case STEP_FULL:      return 1;
        case STEP_HALF:      return 2;
        case STEP_QUARTER:   return 4;
        case STEP_EIGHTH:    return 8;
        case STEP_SIXTEENTH: return 16;
        default:             return 1;
    }
}


static StepMode_t step_from_divider(int n)
{
    switch(n)
    {
        case 1:  return STEP_FULL;
        case 2:  return STEP_HALF;
        case 4:  return STEP_QUARTER;
        case 8:  return STEP_EIGHTH;
        case 16: return STEP_SIXTEENTH;
        default: return STEP_FULL;
    }
}

//step config is the same for both motors
step_config_st step_configuration(StepMode_t mode)
{
    step_config_st step_config;

    uint32_t clearMask = MS1_Pin | MS2_Pin | MS3_Pin;
    uint32_t setMask   = 0;

    // Monta máscara conforme os bits do enum
    if (mode & 0x01) setMask |= MS1_Pin;   // MS1
    if (mode & 0x02) setMask |= MS2_Pin;   // MS2
    if (mode & 0x04) setMask |= MS3_Pin;   // MS3

    // Limpa e aplica de forma atômica
    MS1_GPIO_Port->BRR  = clearMask;
    MS1_GPIO_Port->BSRR = setMask;

    step_config.mode = mode;

    return step_config;
}


//adjust the rpm in function of motor
//missing limit the rpm and when rpm is 0, stop the motor
void Motor_SetRPM(uint8_t motor, float rpm){
    TIM_TypeDef *TIMx;
    uint32_t channel = LL_TIM_CHANNEL_CH1;

    // Seleciona timer conforme motor
    if (motor == MOTOR_X)
        TIMx = TIM16;
    else if (motor == MOTOR_Y)
        TIMx = TIM17;
    else
        return; // motor inválido

    uint8_t divider = rpm_divider(step_config);  // 1,2,4,8,16

    if (rpm <= 0.0f)
    {
        LL_TIM_CC_DisableChannel(TIMx, channel);
        LL_TIM_OC_SetCompareCH1(TIMx, 0);
        return;
    }

    // ARR = 360000 / (rpm * divider) - 1
    float arr_f = (360000.0f / (rpm * (float)divider)) - 1.0f;

    if (arr_f < 0.0f)
        arr_f = 0.0f;

    uint32_t arr = (uint32_t)(arr_f + 0.5f);

    const uint32_t ccr = 120; // ~100us

    if (arr < ccr)
        arr = ccr;

    LL_TIM_SetAutoReload(TIMx, arr);
    LL_TIM_OC_SetCompareCH1(TIMx, ccr);
    LL_TIM_GenerateEvent_UPDATE(TIMx);

    LL_TIM_CC_EnableChannel(TIMx, channel);
}






dir_status_st motor_dir(uint8_t motor_number, uint8_t direction){
	dir_status_st internal_dir;

	if (direction==RIGHT){
		if (motor_number==MOTOR_X){ MOTOR_X_RIGHT }
		if (motor_number==MOTOR_Y){ MOTOR_Y_RIGHT }

	 }
	if (direction==LEFT){
		if (motor_number==MOTOR_X){ MOTOR_X_LEFT }
		if (motor_number==MOTOR_Y){ MOTOR_Y_LEFT }
	 }


	internal_dir.direction[motor_number]=direction;
	return internal_dir;
}

void reset_driver(void){
	RESET_ON_CTRL;
	LL_mDelay(1);
	RESET_OFF_CTRL;
}



bit_state_st enable_motors(uint8_t status){
	bit_state_st internal;

	if(status)ENABLE_MOTORS
	else DISABLE_MOTORS

	internal.status=status;
	return internal;


}


//parser


/*
cmd_t parse_line(const char *s)
{
    cmd_t c = {0, 0, 0};

    // Pula espaços
    while (*s == ' ') s++;

    // Comando deve começar com 'V'
    if (*s != 'V' && *s != 'v') return c;
    s++;

    // Espera pelo menos um espaço
    while (*s == ' ') s++;
    if (*s == '\0') return c;

    // rpmX
    char *endp;
    long x = strtol(s, &endp, 10);
    if (endp == s) return c;      // não leu número
    s = endp;

    while (*s == ' ') s++;
    if (*s == '\0') return c;

    // rpmY
    long y = strtol(s, &endp, 10);
    if (endp == s) return c;

    // (Opcional) validar faixa
    if (x < -2000) x = -2000;
    if (x >  2000) x =  2000;
    if (y < -2000) y = -2000;
    if (y >  2000) y =  2000;

    c.rpm_x = (int16_t)x;
    c.rpm_y = (int16_t)y;
    c.valid = 1;
    return c;
}


void apply_cmd(const cmd_t *c){
    // X
    if (c->rpm_x == 0) {
        Motor_SetRPM(MOTOR_X, 0);
    } else {
        if (c->rpm_x > 0) {

            motor_direction=motor_dir(MOTOR_X, RIGHT);
            Motor_SetRPM(MOTOR_X, (float)c->rpm_x);
        } else {
            motor_direction=motor_dir(MOTOR_X, LEFT);
            Motor_SetRPM(MOTOR_X, (float)(-c->rpm_x));
        }
    }

    // Y
    if (c->rpm_y == 0) {
        Motor_SetRPM(MOTOR_Y, 0);
    } else {
        if (c->rpm_y > 0) {
            motor_direction=motor_dir(MOTOR_Y, RIGHT);
            Motor_SetRPM(MOTOR_Y, (float)c->rpm_y);
        } else {
            motor_direction=motor_dir(MOTOR_Y, LEFT);
            Motor_SetRPM(MOTOR_Y, (float)(-c->rpm_y));
        }
    }
}

*/

cmd_t parse_line(const char *s)
{
    cmd_t c = {0};
    c.valid = 0;
    c.type = CMD_NONE;

    while (*s == ' ') s++;
    if (*s == '\0') return c;

    // ----- Comando V: velocidades -----
    if (*s == 'V' || *s == 'v')
    {
        s++;
        while (*s == ' ') s++;
        if (*s == '\0') return c;

        char *endp;
        long x = strtol(s, &endp, 10);
        if (endp == s) return c;
        s = endp;

        while (*s == ' ') s++;
        if (*s == '\0') return c;

        long y = strtol(s, &endp, 10);
        if (endp == s) return c;

        if (x < -2000) x = -2000;
        if (x >  2000) x =  2000;
        if (y < -2000) y = -2000;
        if (y >  2000) y =  2000;

        c.type  = CMD_VEL;
        c.rpm_x = (int16_t)x;
        c.rpm_y = (int16_t)y;
        c.valid = 1;
        return c;
    }

    // ----- Comando S: step mode -----
    if (*s == 'S' || *s == 's')
    {
        s++;
        while (*s == ' ') s++;
        if (*s == '\0') return c;

        char *endp;
        long n = strtol(s, &endp, 10);
        if (endp == s) return c;

        // aceita somente 1/2/4/8/16
        if (!(n == 1 || n == 2 || n == 4 || n == 8 || n == 16))
            return c;

        c.type = CMD_STEP;
        c.step_mode = step_from_divider((int)n);
        c.valid = 1;
        return c;
    }

    return c;
}

void apply_cmd(const cmd_t *c)
{
    if (!c || !c->valid) return;

    if (c->type == CMD_STEP)
    {
        // Sugestão segura: pare motores antes de mudar o passo
        Motor_SetRPM(MOTOR_X, 0);
        Motor_SetRPM(MOTOR_Y, 0);

        // aplica nos pinos MS1/MS2/MS3 e guarda config
        step_config = step_configuration(c->step_mode);
        return;
    }

    if (c->type == CMD_VEL)
    {
        // X
        if (c->rpm_x == 0) {
            Motor_SetRPM(MOTOR_X, 0);
        } else if (c->rpm_x > 0) {
            motor_direction = motor_dir(MOTOR_X, RIGHT);
            Motor_SetRPM(MOTOR_X, (float)c->rpm_x);
        } else {
            motor_direction = motor_dir(MOTOR_X, LEFT);
            Motor_SetRPM(MOTOR_X, (float)(-c->rpm_x));
        }

        // Y
        if (c->rpm_y == 0) {
            Motor_SetRPM(MOTOR_Y, 0);
        } else if (c->rpm_y > 0) {
            motor_direction = motor_dir(MOTOR_Y, RIGHT);
            Motor_SetRPM(MOTOR_Y, (float)c->rpm_y);
        } else {
            motor_direction = motor_dir(MOTOR_Y, LEFT);
            Motor_SetRPM(MOTOR_Y, (float)(-c->rpm_y));
        }
    }
}

