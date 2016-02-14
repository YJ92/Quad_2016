#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_i2c.h"
#include "driverlib/i2c.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/fpu.h"
#include "driverlib/debug.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"

#include "my_api/Configure_system.h"
#include "my_api/I2C_API.h"
#include "my_api/Kalman_Filter.h"
#include "my_api/Motor_Matrix.h"
#include "my_api/Controller.h"
#include "my_api/Kalman_Filter.h"
#include "my_api/My_utils.h"
#include "my_api/PID_v1.h"
#include "my_api/variables_map.h"


void main(void) {
	/***********************************/
	//1.Setting system clock           //
	/***********************************/
	SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

	FPULazyStackingEnable();
	/*****************************************/
	//2.Configuration UART, I2C, Timer, PWM  //
	/*****************************************/

	// peripherals
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	// ESC off
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0);

	// Xbee on
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_3,GPIO_PIN_3);

	ConfigureUART();
	I2C0_Setup();
	ConfigureTimer();
	ConfigurePWM();


	_flag_200 = false;
	_count_200 = 0;
	float hover = 10;
	int sample_time = 0;
	char buff;
	float rate_increa = 0;
	float ef_desire_angle[3] = {0.0, 0.0, 0.0};

	_gyro_offset[0] = 0.0;
	_gyro_offset[1] = 0.0;
	_gyro_offset[2] = 0.0;

	_q_offset[0] = 1.0;
	_q_offset[1] = 0.0;
	_q_offset[2] = 0.0;
	_q_offset[3] = 0.0;

	/***********************************/
	//3. Device Setup                  //
	/***********************************/
	MPU6050_on();
	ESC_on();

	while(sample_time < 200){
		SysCtlDelay(80000000/3*0.01);
		GetFromMPU6050(_w_gyro, 0);
			_gyro_offset[0] += _w_gyro[0];
			_gyro_offset[1] += _w_gyro[1];
			_gyro_offset[2] += _w_gyro[2];

			sample_time += 1;
	}

	_gyro_offset[0] /= 200.0;
	_gyro_offset[1] /= 200.0;
	_gyro_offset[2] /= 200.0;

	UARTprintf("Offset correction? ");
	while(!UARTCharsAvail(UART1_BASE)){};

	sample_time = 0;

	while(1){
		if(_flag_200){
			_flag_200 = false;
			KalmanFilter();
			sample_time += 1;
		}
		if(sample_time > 200)
			break;
	};

	if(UARTCharsAvail(UART1_BASE)){
		UARTprintf("Offset? ");
		buff = UARTCharGet(UART1_BASE);
		if(buff == '1'){
			_q_offset[0] = _quaternion[0];
			_q_offset[1] = _quaternion[1];
			_q_offset[2] = _quaternion[2];
			_q_offset[3] = _quaternion[3];
		}
	}
	UARTprintf("%d   %d   %d   %d\n",(int)(_q_offset[0]*1000.0),(int)(_q_offset[1]*1000.0),(int)(_q_offset[2]*1000.0),(int)(_q_offset[3]*1000.0));
	UARTprintf("Start");


	UARTprintf("Motor armed...");
	Motor_write(front_motor,10);
	Motor_write(left_motor,10);
	Motor_write(right_motor,10);
	Motor_write(back_motor,10);
	UARTprintf("Complete...\n");
	/***********************************/
	//4. Data Get                      //
	/***********************************/




	Init_PID();
	_count_200 = 0;

	SysCtlDelay(80000000/3*0.5);
	while(1){
		if(_flag_200){ // 200Hz flag
		_flag_200 = false;
		KalmanFilter();
		//stablize_mode(ef_desire_angle[0], ef_desire_angle[1], 0.0, hover);
		rate_controller(ef_desire_angle,1, hover);
		}

		if(_count_200%5 == 0)
			//UARTprintf("%d  %d\n",(int)_right_out, (int)_left_out);

		if(UARTCharsAvail(UART1_BASE)){
			buff = UARTCharGet(UART1_BASE);
			if(buff == '6')
				ef_desire_angle[0] += 10;
			else if(buff == '4')
				ef_desire_angle[0] -= 10;
			else if(buff == '8')
				hover += 1;
			else if(buff == '2')
				hover -= 1;
			else if(buff == '5'){
				Motor_write(front_motor,0);
				Motor_write(left_motor,0);
				Motor_write(right_motor,0);
				Motor_write(back_motor,0);
				break;
			}else
				rate_increa = 0.0;

			UARTprintf("angle : %d , hover : %d\n",(int)ef_desire_angle[0],(int)hover);
		}

		if(_count_200 > 5500){
			//break;
		}
	}

	//release();
	//GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_0,0);
	while(1){
		Motor_write(front_motor,0);
		Motor_write(left_motor,0);
		Motor_write(right_motor,0);
		Motor_write(back_motor,0);
	}

}
