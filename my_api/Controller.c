/*
 * Controller.c
 *
 *  Created on: 2016. 2. 12.
 *      Author: YJ
 */
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/pwm.h"

#include "I2C_API.h"
#include "My_utils.h"
#include "Motor_Matrix.h"
#include "Controller.h"
#include "PID_v1.h"
#include "variables_map.h"

PidType _r_rate_PID, _p_rate_PID, _y_rate_PID;

float _r_rate_out, _p_rate_out, _y_rate_out;
float _front_out, _left_out, _right_out, _back_out;

//*****************************************************************************
// PID control : 200Hz
//*****************************************************************************
void Init_PID(){
	// Rate pid
	PID_init(&_r_rate_PID, r_rate_Kp, r_rate_Ki, r_rate_Kd, PID_Direction_Direct);
	PID_init(&_p_rate_PID, p_rate_Kp, p_rate_Ki, p_rate_Kd, PID_Direction_Direct);
	PID_init(&_y_rate_PID, y_rate_Kp, y_rate_Ki, y_rate_Kd, PID_Direction_Direct);

	PID_SetSampleTime(&_r_rate_PID, 5); // 0.005s
	PID_SetSampleTime(&_p_rate_PID, 5);
	PID_SetSampleTime(&_y_rate_PID, 5);

	PID_SetMode(&_r_rate_PID, PID_Mode_Automatic);
	PID_SetMode(&_p_rate_PID, PID_Mode_Automatic);
	PID_SetMode(&_y_rate_PID, PID_Mode_Automatic);

	PID_SetOutputLimits(&_r_rate_PID,-20.0,20.0);
	PID_SetOutputLimits(&_p_rate_PID,-20.0,20.0);
	PID_SetOutputLimits(&_y_rate_PID,-20.0,20.0);
}

// input : deg/s
// rate_type : 1-> earth frame desire rate(roll, pitch, yaw)
// rate_type : 0-> body frame desire rate(p,q,r)
void rate_controller(float* desire_rate,float* attitude, unsigned long rate_type){
	float w[3];
	GetFromMPU6050(w, 0);
	_r_rate_PID.myInput = w[0]*180.0/3.14;  // deg/s
	_p_rate_PID.myInput = w[1]*180.0/3.14;
	_y_rate_PID.myInput = w[2]*180.0/3.14;

	if(rate_type == 1){
		// earth frame to body frame
		_r_rate_PID.mySetpoint = desire_rate[0] - sinf(attitude[1])*desire_rate[2];
		_p_rate_PID.mySetpoint =  cosf(attitude[0])*desire_rate[1] + sinf(attitude[0])*cosf(attitude[1])*desire_rate[2];
		_y_rate_PID.mySetpoint = -sinf(attitude[1])*desire_rate[1] + cosf(attitude[0])*sinf(attitude[1])*desire_rate[2];
	}else{
		_r_rate_PID.mySetpoint = desire_rate[0];
		_p_rate_PID.mySetpoint = desire_rate[1];
		_y_rate_PID.mySetpoint = desire_rate[2];
	}

	PID_Compute(&_r_rate_PID);
	PID_Compute(&_p_rate_PID);
	PID_Compute(&_y_rate_PID);

	_r_rate_out = _r_rate_PID.myOutput;
	_p_rate_out = _p_rate_PID.myOutput;
	_y_rate_out = _y_rate_PID.myOutput;

	_front_out  = 10              - _p_rate_out  - _y_rate_out;
	_right_out  = 10 - _r_rate_out               + _y_rate_out;
	_back_out   = 10              + _p_rate_out  - _y_rate_out;
	_left_out   = 10 + _r_rate_out               + _y_rate_out;

	_front_out = constraints(_front_out,10, 50);
	_right_out = constraints(_right_out,10, 50);
	_back_out = constraints(_back_out,10, 50);
	_left_out = constraints(_left_out,10, 50);

	Motor_write(front_motor, _front_out);
	Motor_write(right_motor, _right_out);
	Motor_write(back_motor, _back_out);
	Motor_write(left_motor, _left_out);


}

// input : degree
void stablize_mode(float* ef_desire_angle, float* n_q){
	float d_q[4], e_q[4];
	float bf_desire_rate[3];
	float norm, angle_error;
	float phi = ef_desire_angle[0]*0.008726646;  // degree to radian
	float theta = ef_desire_angle[1]*0.008726646;
	float psi = ef_desire_angle[2]*0.008726646;

	float c_phi = cosf(phi); float s_phi = sinf(phi);
	float c_theta = cosf(theta); float s_theta = sinf(theta);
	float c_psi = cosf(psi); float s_psi = sinf(psi);

	d_q[0] = c_phi*c_theta*c_psi + s_phi*s_theta*s_psi;
	d_q[1] = s_phi*c_theta*c_psi - c_phi*s_theta*s_psi;
	d_q[2] = c_phi*s_theta*c_psi + s_phi*c_theta*s_psi;
	d_q[3] = c_phi*c_theta*s_psi - s_phi*s_theta*c_psi;

	e_q[0] = d_q[0]*n_q[0] + d_q[1]*n_q[1] + d_q[2]*n_q[2] + d_q[3]*n_q[3];
	e_q[1] = d_q[1]*n_q[0] - d_q[0]*n_q[1] + d_q[3]*n_q[2] - d_q[2]*n_q[3];
	e_q[2] = d_q[2]*n_q[0] - d_q[3]*n_q[1] - d_q[0]*n_q[2] + d_q[1]*n_q[3];
	e_q[3] = d_q[3]*n_q[0] + d_q[2]*n_q[1] - d_q[1]*n_q[2] - d_q[0]*n_q[3];

	norm = sqrtf(e_q[0]*e_q[0]+e_q[1]*e_q[1]+e_q[2]*e_q[2]+e_q[3]*e_q[3]);
	e_q[0] /= norm;

	if(e_q[0] < 0){
		e_q[0] *= -1;
		e_q[1] *= -1;
		e_q[2] *= -1;
		e_q[3] *= -1;
	}

	angle_error = acosf(e_q[0]);

	norm = sqrtf(e_q[1]*e_q[1]+e_q[2]*e_q[2]+e_q[3]*e_q[3]);
	bf_desire_rate[0] = e_q[1]/norm;
	bf_desire_rate[1] = e_q[2]/norm;
	bf_desire_rate[2] = e_q[3]/norm;


	angle_error = stable_Kp*angle_error*114.591559; // Kp*2*angle*180/3.14
	constraints(angle_error,0.0, 20.0);  // max 20deg/s

	bf_desire_rate[0] *= angle_error;
	bf_desire_rate[1] *= angle_error;
	//bf_desire_rate[2] *= angle_error;
	bf_desire_rate[2] = 0;

	rate_controller(bf_desire_rate, bf_desire_rate, 0);
}
