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


step_config_st step_configuration(uint8_t value);
dir_status_st motor_dir(uint8_t motor_number, uint8_t direction);
bit_state_st sleep_driver(uint8_t status);
bit_state_st enable_motors(uint8_t status);
void reset_driver(void);
#endif /* SRC_SPECIAL_H_ */
