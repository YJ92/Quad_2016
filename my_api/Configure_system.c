/*
 * Configure_system.c
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
#include "driverlib/interrupt.h"
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

#include "Motor_Matrix.h"
#include "Configure_system.h"
#include "PID_v1.h"
#include "variables_map.h"

unsigned long _flag;
int _count;

//*****************************************************************************
// PWM Setting(Use PF1 port, M1PWM5) : 500Hz pwm
//*****************************************************************************
void ConfigurePWM(void){

	SysCtlPWMClockSet(SYSCTL_PWMDIV_4);

	// Enable the peripherals
	//SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	//SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	//SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

	// Configure Gpio pin as PWM
	GPIOPinConfigure(GPIO_PB6_M0PWM0);
	GPIOPinConfigure(GPIO_PB4_M0PWM2);
	GPIOPinConfigure(GPIO_PE4_M0PWM4);
	GPIOPinConfigure(GPIO_PC4_M0PWM6);
	GPIOPinTypePWM(GPIO_PORTB_BASE,GPIO_PIN_6|GPIO_PIN_4);
	GPIOPinTypePWM(GPIO_PORTE_BASE,GPIO_PIN_4);
	GPIOPinTypePWM(GPIO_PORTC_BASE,GPIO_PIN_4);


	// Set the period
	PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN|PWM_GEN_MODE_NO_SYNC);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN|PWM_GEN_MODE_NO_SYNC);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN|PWM_GEN_MODE_NO_SYNC);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN|PWM_GEN_MODE_NO_SYNC);

	PWMGenPeriodSet(PWM0_BASE,PWM_GEN_0,PWMPeriod);
	PWMGenPeriodSet(PWM0_BASE,PWM_GEN_1,PWMPeriod);
	PWMGenPeriodSet(PWM0_BASE,PWM_GEN_2,PWMPeriod);
	PWMGenPeriodSet(PWM0_BASE,PWM_GEN_3,PWMPeriod);


	// Set initial duty - 50%
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_0,0);
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,0);
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_4,0);
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_6,0);

	// Enable the PWM generator
//	PWMGenEnable(PWM0_BASE, PWM_GEN_0|PWM_GEN_1|PWM_GEN_2|PWM_GEN_3);
	PWMGenEnable(PWM0_BASE, PWM_GEN_0);
	PWMGenEnable(PWM0_BASE, PWM_GEN_1);
	PWMGenEnable(PWM0_BASE, PWM_GEN_2);
	PWMGenEnable(PWM0_BASE, PWM_GEN_3);

	// Turn on the output pins
//	PWMOutputState(PWM0_BASE,PWM_OUT_0_BIT|PWM_OUT_2_BIT|PWM_OUT_4_BIT|PWM_OUT_6_BIT,true);
	PWMOutputState(PWM0_BASE,PWM_OUT_0_BIT,true);
	PWMOutputState(PWM0_BASE,PWM_OUT_2_BIT,true);
	PWMOutputState(PWM0_BASE,PWM_OUT_4_BIT,true);
	PWMOutputState(PWM0_BASE,PWM_OUT_6_BIT,true);
}

//*****************************************************************************
// I2C0 Setting(Use PB2 : SCL , PB3 : SDA )
//*****************************************************************************

void I2C0_Setup()
{

	// I2C0 Peripheral
	//SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
	GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

	GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	GPIOPinConfigure(GPIO_PB3_I2C0SDA);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

	// Enable and initialize the I2C0 master module.
	// Use System Clock
	// false : normal mode(100kbps), ture : fast mode(400kbps)
	I2CMasterInitExpClk(I2C0_MASTER_BASE, SysCtlClockGet(),false);

}



//*****************************************************************************
// UART Setting(Use PB0,PB1)
//*****************************************************************************
void ConfigureUART(void){
	/// UART1 Enable
	// Enable the peripherals
	//SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

	// Configure Gpio pin as UART
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);
	GPIOPinTypeUART(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1);

	// Properties setting
	UARTClockSourceSet(UART1_BASE,UART_CLOCK_SYSTEM);
	UARTStdioConfig(1,9600,g_uiSysClock);
}

//*****************************************************************************
// Timer Setting(Use PB6) : 200Hz
//*****************************************************************************
void ConfigureTimer(void){

	// Enable the peripherals
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

	// Setting period
	TimerConfigure(TIMER0_BASE, TIMER_CFG_A_PERIODIC);
	TimerPrescaleSet(TIMER0_BASE, TIMER_A, 20);
	TimerLoadSet(TIMER0_BASE, TIMER_A, g_uiSysClock/freq); // 200hz
	// Setting Inturrupt
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0AIntHandler);
	IntEnable(INT_TIMER0A);
}

//*****************************************************************************
// Timer Interrupt
//*****************************************************************************
void Timer0AIntHandler(){
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	_flag = true;
	_count += 1;
	if(_count == 10000)
		_count = 0;

}

