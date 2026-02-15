/*
 * special.h
 *
 *  Created on: Dec 2, 2025
 *      Author: f.ferreira
 */

#ifndef SRC_SPECIAL_H_
#define SRC_SPECIAL_H_

#include "special.h"
#include "stdio.h"
#include "math.h"
#include "main.h"


extern relay_st relay_on_off(uint8_t control);
extern int _write(int file, char *ptr, int len);
extern void print_float(float number);
extern void print_float_1dec(float number);
extern void sort(float* data, int n);
extern float trimmed_mean(float* data, int n, float trim_percentage);


uint8_t rpm_divider(step_config_st step_config);
step_config_st step_configuration(StepMode_t mode);
dir_status_st motor_dir(uint8_t motor_number, uint8_t direction);
bit_state_st enable_motors(uint8_t status);
void reset_driver(void);

void Motor_SetRPM(uint8_t motor, float rpm);

cmd_t parse_line(const char *s);
void apply_cmd(const cmd_t *c);
void Motor_RampTask_10ms(void);

#endif /* SRC_SPECIAL_H_ */
