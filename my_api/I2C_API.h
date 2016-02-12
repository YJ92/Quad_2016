/*
 * I2C_API.h
 *
 *  Created on: 2016. 2. 12.
 *      Author: YJ
 */

#ifndef I2C_API_H_
#define I2C_API_H_

//*****************************************************************************
//
// Additional Defines for the API.
//
//*****************************************************************************

extern unsigned long I2CRegRead(unsigned char ucSlaveAdress, unsigned char ucReg);
extern unsigned long I2CRegWrite(unsigned char ucSlaveAdress, unsigned char ucReg, unsigned char ucValue);
extern void GetFromMPU6050(float* Data,unsigned long ulDataType);
#endif /* I2C_API_H_ */
