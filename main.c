#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_i2c.h"
#include "driverlib/i2c.h"
#include "driverlib/gpio.h"
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

#include "my_api/I2C_API.h"
#include "my_api/Kalman_Filter.h"
#include "my_api/Motor_Matrix.h"
#include "my_api/My_utils.h"
#include "my_api/Controller.h"
#include "my_api/Configure_system.h"
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

	// Xbee on
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_3,GPIO_PIN_3);

	ConfigureUART();
	I2C0_Setup();
	ConfigureTimer();
	ConfigurePWM();

	_flag = false;
	_count = 0;

	/***********************************/
	//3. Device Setup                  //
	/***********************************/
	MPU6050_on();
	ESC_on();


	UARTprintf("Motor armed...\n");
	Motor_write(front_motor,10);
	Motor_write(left_motor,10);
	Motor_write(right_motor,10);
	Motor_write(back_motor,10);
	UARTprintf("Complete...\n");

	// Timer enlabe
	TimerEnable(TIMER0_BASE, TIMER_A);
	while(_count < 200){};
	UARTprintf("Start");



	/***********************************/
	//4. Data Get                      //
	/***********************************/
	float euler_angle[3];
	float quaternion[4];
	char buff;
	float rate_increa = 0;
	float ef_desire_angle[3] = {0.0, 0.0, 0.0};


	Init_PID();

	while(1){
		if(_flag){ // 200Hz flag
		_flag = false;
		EulerAngle(euler_angle, quaternion);
		stablize_mode(ef_desire_angle, quaternion);
		}

		if(_count%5 == 0)
			UARTprintf("%d  %d\n",(int)_front_out, (int)_back_out);

		if(UARTCharsAvail(UART1_BASE)){
			buff = UARTCharGet(UART1_BASE);
			if(buff == '8')
				rate_increa = 5.0;
			else if(buff == '2')
				rate_increa = -5.0;
			else if(buff == '5'){
				Motor_write(front_motor,0);
				Motor_write(left_motor,0);
				Motor_write(right_motor,0);
				Motor_write(back_motor,0);
				break;
			}else
				rate_increa = 0.0;

			ef_desire_angle[1] += rate_increa;
			UARTprintf("%d\n",(int)ef_desire_angle[1]);
		}

		if(_count > 9000){
			Motor_write(front_motor,0);
			Motor_write(left_motor,0);
			Motor_write(right_motor,0);
			Motor_write(back_motor,0);
			break;
		}
	}

	while(1){

	}

}
