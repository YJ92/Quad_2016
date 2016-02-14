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

#define front_factor	0.95
#define right_factor	1.0
#define left_factor		1.13
#define back_factor		1.17
//********************* PID variables******************************************
#define stable_Kp 2
#define r_rate_Kp	0.4
#define r_rate_Ki	0.03
#define r_rate_Kd	0.0
#define p_rate_Kp	0.4
#define p_rate_Ki	0.03
#define p_rate_Kd	0.0
#define y_rate_Kp	0.4
#define y_rate_Ki	0.03
#define y_rate_Kd	0.0


//********************Global variables*****************************************

// used in Controller.c & main.c
extern PidType _r_rate_PID, _p_rate_PID, _y_rate_PID;
extern float _r_rate_out, _p_rate_out, _y_rate_out;
extern float _front_out, _left_out, _right_out, _back_out;
extern float _prev_angle_error;

// used in Configure_system.c & main.c
extern volatile unsigned long _flag_200;
extern volatile int _count_200;

// used in Kalman_Filter.c & main.c
extern float _quaternion[4];
extern float _q_offset[4];
extern float _gyro_offset[3];
extern float _euler_angle[3];
extern float _w_gyro[3];

#endif /* VARIABLES_MAP_H_ */
