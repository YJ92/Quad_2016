/*
 * I2C_API.c
 *
 *  Created on: 2016. 2. 12.
 *      Author: YJ
 */
//*****************************************************************************
// Reads the I2C slave register.
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/i2c.h"
#include "driverlib/rom.h"
#include "I2C_API.h"
#include "PID_v1.h"
#include "variables_map.h"

unsigned long I2CRegRead(unsigned char ucSlaveAdress, unsigned char ucReg){

	unsigned long ulRegValue = 0;

	// Wait until master module is done transferring.
	while(ROM_I2CMasterBusy(I2C0_MASTER_BASE))
	{
	};


    // Tell the master module what address it will place on the bus when
    // 0 : write, 1 : read
    ROM_I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, ucSlaveAdress, 0);


    // Place the command to be sent in the data register.
    ROM_I2CMasterDataPut(I2C0_MASTER_BASE, ucReg);


    // Initiate send of data from the master.
    ROM_I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_SINGLE_SEND);


    // Wait until master module is done transferring.
    while(ROM_I2CMasterBusy(I2C0_MASTER_BASE))
    {
    };


    // Check for errors.
    if(ROM_I2CMasterErr(I2C0_MASTER_BASE) != I2C_MASTER_ERR_NONE)
    {
        return 0;
    }


    // Tell the master module what address it will place on the bus when
    // 0 : write, 1 : read
    ROM_I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, ucSlaveAdress, 1);


    // Tell the master to read data.
    ROM_I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);


    // Wait until master module is done receiving.
    while(ROM_I2CMasterBusy(I2C0_MASTER_BASE))
    {
    };


    // Check for errors.
    if(ROM_I2CMasterErr(I2C0_MASTER_BASE) != I2C_MASTER_ERR_NONE)
    {
        return 0;
    }


    // Read the data from the master.
    ulRegValue = ROM_I2CMasterDataGet(I2C0_MASTER_BASE);


    // Return the register value.
    return ulRegValue;
}

//*****************************************************************************
// Writes to the specified I2C slave register.
//*****************************************************************************

unsigned long I2CRegWrite(unsigned char ucSlaveAdress, unsigned char ucReg, unsigned char ucValue)
{

	// Wait until master module is done transferring.
	while(ROM_I2CMasterBusy(I2C0_MASTER_BASE))
	{
	};

    // Tell the master module what address it will place on the bus when
    // 0 : write, 1 : read
    ROM_I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, ucSlaveAdress, 0);


    // Place the command to be sent in the data register.
    ROM_I2CMasterDataPut(I2C0_MASTER_BASE, ucReg);


    // Initiate send of data from the master.
    ROM_I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);


    // Wait until master module is done transferring.
    while(ROM_I2CMasterBusy(I2C0_MASTER_BASE))
    {
    };


    // Check for errors.
    if(ROM_I2CMasterErr(I2C0_MASTER_BASE) != I2C_MASTER_ERR_NONE)
    {
        return 0;
    }


    // Place the value to be sent in the data register.
    ROM_I2CMasterDataPut(I2C0_MASTER_BASE, ucValue);


    // Initiate send of data from the master.
    ROM_I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);


    // Wait until master module is done transferring.
    while(ROM_I2CMasterBusy(I2C0_MASTER_BASE))
    {
    };


    // Check for errors.
    if(ROM_I2CMasterErr(I2C0_MASTER_BASE) != I2C_MASTER_ERR_NONE)
    {
        return 0;
    }


    // Initiate send of data from the master.
    ROM_I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);


    // Wait until master module is done transferring.
    while(ROM_I2CMasterBusy(I2C0_MASTER_BASE))
    {
    };


    // Check for errors.
    if(ROM_I2CMasterErr(I2C0_MASTER_BASE) != I2C_MASTER_ERR_NONE)
    {
        return 0;
    }


    // Return 1 if there is no error.
    return 1;
}


//*****************************************************************************
// Get Data from MPU6050
// ulDataType = true : return acc, false : return gyro
//*****************************************************************************

void GetFromMPU6050(float* Data,unsigned long ulDataType){

	unsigned char Data_H[3], Data_L[3];
	short buff;
	int i;

	// Get High and Low data byte
	if(ulDataType){
			// Accelerometer Measurements
			Data_H[0] = I2CRegRead(0x68,0x3B);
			Data_L[0] = I2CRegRead(0x68,0x3C);
			Data_H[1] = I2CRegRead(0x68,0x3D);
			Data_L[1] = I2CRegRead(0x68,0x3E);
			Data_H[2] = I2CRegRead(0x68,0x3F);
			Data_L[2] = I2CRegRead(0x68,0x40);

			for(i=0;i<3;i++){
				buff = Data_H[i]*256+Data_L[i];
				Data[i] = (float)((buff/16384.0)*9.8);
			}
	}else{
			// Gyroscope Measurements
			Data_H[0] = I2CRegRead(0x68,0x43);
			Data_L[0] = I2CRegRead(0x68,0x44);
			Data_H[1] = I2CRegRead(0x68,0x45);
			Data_L[1] = I2CRegRead(0x68,0x46);
			Data_H[2] = I2CRegRead(0x68,0x47);
			Data_L[2] = I2CRegRead(0x68,0x48);

			for(i=0;i<3;i++){
				buff = Data_H[i]*256+Data_L[i];
				Data[i] = (float)((buff/131.0)*(3.14/180));
			}
	}

}
