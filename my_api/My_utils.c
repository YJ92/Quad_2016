/*
 * My_utils.c
 *
 *  Created on: 2016. 2. 12.
 *      Author: YJ
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_i2c.h"
#include "driverlib/i2c.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"

#include "I2C_API.h"
#include "My_utils.h"
#include "Motor_Matrix.h"
#include "PID_v1.h"
#include "variables_map.h"


float constraints(float input, float min, float max){
	if(input < min)
		return min;

	if(input > max)
		return max;
	return input;
}

void ESC_on(){
	UARTprintf("ESC on...");
	Motor_write(front_motor,0);
	Motor_write(left_motor,0);
	Motor_write(right_motor,0);
	Motor_write(back_motor,0);
	SysCtlDelay(80000000/3*0.1);
	GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_0,GPIO_PIN_0);
	SysCtlDelay(80000000/3*5);
	UARTprintf("Complete...\n");
}

void MPU6050_on(){
	UARTprintf("MPU6050 setting...");
	// MPU6050 => Clear the 'sleep' bit to start the sensor
	I2CRegWrite(0x68,0x6B,0);
	SysCtlDelay(80000000/3);
	I2CRegWrite(0x68,0x6B,0);
	SysCtlDelay(8000000/3);
	UARTprintf("Complete...\n");
}
