/*
 * Controller.h
 *
 *  Created on: 2016. 2. 12.
 *      Author: YJ
 */

#ifndef MY_API_CONTROLLER_H_
#define MY_API_CONTROLLER_H_

extern void Init_PID(void);
extern void rate_controller(float* desire_rate,float* attitude, unsigned long rate_type);
extern void stablize_mode(float* ef_desire_angle, float* n_q);


#endif /* MY_API_CONTROLLER_H_ */
