/*
 * variables_map.h
 *
 *  Created on: 2016. 2. 12.
 *      Author: YJ
 */

#ifndef VARIABLES_MAP_H_
#define VARIABLES_MAP_H_

#define I2C0_MASTER_BASE        0x40020000  // I2C0 Master
#define I2C0_SLAVE_BASE         0x40020800  // I2C0 Slave

//********************Kalman filter variables**********************************
#define g_uiSysClock	80000000
#define Q_q	0.003
#define R_r	0.5
#define dt	0.005
#define freq 200
#define PWMPeriod 40000	// g_uiSysClock/PWMFreq/4; // 500Hz pwm
//#define PWMFreq	500

//********************* motor variables ***************************************
#define front_motor PWM_OUT_2
#define right_motor PWM_OUT_6
#define back_motor PWM_OUT_4
#define left_motor	PWM_OUT_0

//********************* PID variables******************************************
#define stable_Kp 1
#define r_rate_Kp	1
#define r_rate_Ki	0
#define r_rate_Kd	0
#define p_rate_Kp	1
#define p_rate_Ki	0
#define p_rate_Kd	0
#define y_rate_Kp	1
#define y_rate_Ki	0
#define y_rate_Kd	0


//********************Global variables*****************************************

// used in Controller.c & main.c
extern PidType _r_rate_PID, _p_rate_PID, _y_rate_PID;
extern float _r_rate_out, _p_rate_out, _y_rate_out;
extern float _front_out, _left_out, _right_out, _back_out;

// used in Configure_system.c & main.c
extern unsigned long _flag;
extern int _count;

#endif /* VARIABLES_MAP_H_ */
