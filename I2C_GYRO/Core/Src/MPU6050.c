/*
 * MPU6050.cpp
 *
 *  Created on: Jul 12, 2025
 *      Author: Tushar
 */


#include "MPU6050.h"
#include "I2Cdev.h"
#include <stdio.h>

uint16_t getFIFOCount() {
	uint8_t buffer[2];
	uint8_t count;
    readBytes(devAddr, MPU6050_RA_FIFO_COUNTH, 2, &buffer[0], 1000);

    return ((((uint16_t)buffer[0])<<8)|buffer[1]);
}

/*
int8_t MPU6050_Base::GetCurrentFIFOPacket(uint8_t *data, uint8_t length) { // overflow proof
     int16_t fifoC;
     // This section of code is for when we allowed more than 1 packet to be acquired
     uint32_t BreakTimer = micros();
     bool packetReceived = false;
     do {
         if ((fifoC = getFIFOCount())  > length) {

             if (fifoC > 200) { // if you waited to get the FIFO buffer to > 200 bytes it will take longer to get the last packet in the FIFO Buffer than it will take to  reset the buffer and wait for the next to arrive
                 resetFIFO(); // Fixes any overflow corruption
                 fifoC = 0;
                 while (!(fifoC = getFIFOCount()) && ((micros() - BreakTimer) <= (getFIFOTimeout()))); // Get Next New Packet
                 } else { //We have more than 1 packet but less than 200 bytes of data in the FIFO Buffer
                 uint8_t Trash[I2CDEVLIB_WIRE_BUFFER_LENGTH];
                 while ((fifoC = getFIFOCount()) > length) {  // Test each time just in case the MPU is writing to the FIFO Buffer
                     fifoC = fifoC - length; // Save the last packet
                     uint16_t  RemoveBytes;
                     while (fifoC) { // fifo count will reach zero so this is safe
                         RemoveBytes = (fifoC < I2CDEVLIB_WIRE_BUFFER_LENGTH) ? fifoC : I2CDEVLIB_WIRE_BUFFER_LENGTH; // Buffer Length is different than the packet length this will efficiently clear the buffer
                         getFIFOBytes(Trash, (uint8_t)RemoveBytes);
                         fifoC -= RemoveBytes;
                     }
                 }
             }
         }
         if (!fifoC) return 0; // Called too early no data or we timed out after FIFO Reset
         // We have 1 packet
         packetReceived = fifoC == length;
         if (!packetReceived && (micros() - BreakTimer) > (getFIFOTimeout())) return 0;
     } while (!packetReceived);
     getFIFOBytes(data, length); //Get 1 packet
     return 1;
}
*/
uint8_t MPU6050_Init(MPU6050 *dev, I2C_HandleTypeDef *i2cHandle) {

    dev->i2cHandle = i2cHandle;

    for (int n = 0; n < 3; n++) {

    dev->acc_mps2[n] = 0.0f;
    dev->gyr_rps[n] = 0.0f;

    dev->temp_degC = 0.0f;
    }

    for (int n = 0; n < 14; n++) {
    dev->rxData[n] = 0;

    }

    /* Clear DMA flag */
    dev->rxFlag = 0;

    dev->dataReadyFlag = 0;

    // Initialise sensor and set default parameters (by default: acc range = +-2g, gyr range = +-250dps)

    uint8_t numErrors = 0;
    HAL_StatusTypeDef status;

    /* Disable FSYNC, enable Digital low-pass filter (fs=1kHz, bandwidth: acc=94Hz, gyr=98Hz) */
    uint8_t data = 0x02;
    status = HAL_I2C_Mem_Write(dev->i2cHandle, MPU6050_I2C_ADDR, MPU6050_REG_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    numErrors += (status != HAL_OK);

    /* Enable interrupt and Enterrupt status bits are cleared on any read operation */
    data = 0x10;
    status = HAL_I2C_Mem_Write(dev->i2cHandle, MPU6050_I2C_ADDR, MPU6050_REG_INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    numErrors += (status != HAL_OK);

    /* */
    data = 0x01;
      status = HAL_I2C_Mem_Write(dev->i2cHandle, MPU6050_I2C_ADDR, MPU6050_REG_INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
      numErrors += (status != HAL_OK);

      /*Wakee up IMU*/
      data = 0x00;
      status = HAL_I2C_Mem_Write(dev->i2cHandle, MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
      numErrors += (status != HAL_OK);


    return numErrors;

}

uint8_t MPU6050_Read_DMA(MPU6050 *dev){
	/* Attempt to start DMA transfer from IMU to memory */
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(dev->i2cHandle, MPU6050_I2C_ADDR, MPU6050_REG_IMU_DATA_START, I2C_MEMADD_SIZE_8BIT, dev->rxData, 14);

	dev->rxFlag = (status == HAL_OK);

	dev->dataReadyFlag = 0;

	return dev->rxFlag;
}

void MPU6050_Process_Data(MPU6050 *dev) {

	/*
	* Combine raw data to signed ints
	*/
	int16_t acc[3] = { ( (dev->rxData[0] << 8) | dev->rxData[1] ),
					   ( (dev->rxData[2] << 8) |dev->rxData[3] ),
					   ( (dev->rxData[4] << 8) | dev->rxData[5] )};

	int16_t temp = (dev->rxData[6] << 8) | dev->rxData[7];

	int16_t gyr[3] = { ( (dev->rxData[8] << 8) | dev->rxData[9] ),
					   ( (dev->rxData[10] << 8) | dev->rxData[11] ),
					   ( (dev->rxData[12] << 8) | dev->rxData[13] )};

	/* Convert signed ints to 'units' (m/s^2, ℃, cad/s)
	*/

	/* Accelerometer ( !!!!!!!!! WITH RE-MAP FOR LITTLEBRAIN++ !!!!!!!! ) */
	dev->acc_mps2[0] = -MPU6050_ACC_RAW_TO_MPS2 * acc[1];
	dev->acc_mps2[1] = -MPU6050_ACC_RAW_TO_MPS2 * acc[0];
	dev->acc_mps2[2] = -MPU6050_ACC_RAW_TO_MPS2 * acc[2];

	/* Temperature */
	dev->temp_degC = MPU6050_TEMP_RAW_TO_DEGC * ( temp + MPU6050_TEMP_DEGC_OFFSET );

	//Gyroscope ( !!!!
	dev->gyr_rps[0] = -MPU6050_GYR_RAW_TO_RPS * gyr[1];
	dev->gyr_rps[1] = -MPU6050_GYR_RAW_TO_RPS * gyr[0];
	dev->gyr_rps[2] = -MPU6050_GYR_RAW_TO_RPS * gyr[2];
}




