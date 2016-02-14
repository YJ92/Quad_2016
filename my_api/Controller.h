/*
 * Controller.h
 *
 *  Created on: 2016. 2. 12.
 *      Author: YJ
 */

#ifndef MY_API_CONTROLLER_H_
#define MY_API_CONTROLLER_H_

extern void Init_PID(void);
extern void rate_controller(float* desire_rate, unsigned long rate_type, float hover);
extern void stablize_mode(float ef_desire_roll, float ef_desire_pitch, float bf_desire_yaw_rate, float hover);


#endif /* MY_API_CONTROLLER_H_ */
