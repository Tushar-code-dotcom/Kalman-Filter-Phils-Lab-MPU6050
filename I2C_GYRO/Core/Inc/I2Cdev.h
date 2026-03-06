/*
 * i2cdev.h
 *
 *  Created on: Jun 30, 2025
 *      Author: Tushar
 */
#include <stdint.h>
#include <stdbool.h>

#ifndef I2Cdev_H_
#define I2Cdev_H_

#define MPU6050_RA_PWR_MGMT_1 0x6B
#define MPU6050_PWR1_CLKSEL_BIT 2
#define MPU6050_PWR1_CLKSEL_LENGTH 3
#define MPU6050_CLOCK_PLL_XGYRO  0x01



bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout);
int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout);
#endif /* MPU6050_H_ */
