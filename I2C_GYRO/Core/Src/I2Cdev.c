/*
 * mpu6050.c
 *
 *  Created on: Jun 30, 2025
 *      Author: Tushar
 */

#include <I2Cdev.h>
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>


extern I2C_HandleTypeDef hi2c1;
#define TIMEOUT 500

bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    readByte(devAddr, regAddr, &b, TIMEOUT);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(devAddr, regAddr, b);
}

bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data){

		//      010 value to write
		// 76543210 bit numbers
		//    xxx   args: bitStart=4, length=3
		// 00011100 mask byte
		// 10101111 original value (sample)
		// 10100011 original & ~mask
		// 10101011 masked | value
		uint8_t b;
		if (readByte(devAddr, regAddr, &b, TIMEOUT) != 0) {
			uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
			data <<= (bitStart - length + 1); // shift data into correct position
			data &= mask; // zero all non-important bits in data
			b &= ~(mask); // zero all important bits in existing byte
			b |= data; // combine data with existing byte
			return writeByte(devAddr, regAddr, b);
	}
}

int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout){
	uint8_t count=0;
	if(HAL_I2C_Mem_Read(&hi2c1, (uint16_t)devAddr, (uint16_t)regAddr, 1, data, (uint16_t)length, (uint32_t)timeout)==HAL_OK){
		count==length;
		return count;
	}
}

int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout) {
    if(HAL_I2C_Mem_Read(&hi2c1, (uint16_t)devAddr, (uint16_t)regAddr, 1, data, 1,(uint32_t)timeout)==HAL_OK){
    return 1;
    }
    else{
    	return 0;
    }
}


/** Write multiple bytes to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data){
	bool ret_val;
	if(HAL_I2C_Mem_Write(&hi2c1, (uint16_t)devAddr, (uint16_t)regAddr, 1, data, (uint16_t)length, TIMEOUT==HAL_OK)){
		ret_val=true;
	}
	else{
		ret_val=false;
	}
	return ret_val;
}

bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    if(HAL_I2C_Mem_Write(&hi2c1, devAddr, regAddr, 1, &data, 1, 1000)==HAL_OK){
    	return true;
    }
    else{
    	return false;
    }
}



