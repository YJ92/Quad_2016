/*
 * Motor_Matrix.c
 *
 *  Created on: 2016. 2. 12.
 *      Author: YJ
 */
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/pwm.h"
#include "Motor_Matrix.h"
#include "PID_v1.h"
#include "variables_map.h"

void Motor_write(uint32_t selected_motor, float percent){
	float duty;

	if(selected_motor != front_motor && selected_motor != left_motor && selected_motor != right_motor && selected_motor != back_motor )
		return;

	if(percent <= 0.0)
		percent = 0.0;

	if(percent >= 100.0)
		percent = 100.0;

	duty = 0.6*percent + 20.0;
	duty = PWMPeriod*duty/100.0;

	PWMPulseWidthSet(PWM0_BASE,selected_motor,duty);
}
