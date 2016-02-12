/*
 * Configure_system.h
 *
 *  Created on: 2016. 2. 12.
 *      Author: YJ
 */

#ifndef MY_API_CONFIGURE_SYSTEM_H_
#define MY_API_CONFIGURE_SYSTEM_H_

extern void ConfigurePWM();
extern void I2C0_Setup();
extern void ConfigureUART();
extern void ConfigureTimer();
extern void Timer0AIntHandler();

#endif /* MY_API_CONFIGURE_SYSTEM_H_ */
