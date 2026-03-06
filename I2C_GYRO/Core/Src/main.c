/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <KalmanRollPitch.h>
#include "MPU6050.h"
#include "I2Cdev.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LPF_GYR_ALPHA 0.01f
#define LPF_ACC_ALPHA 0.10f

#define KALMAN_P_INIT 0.1f
#define KALMAN_Q 0.001f
#define KALMAN_R 0.011f

#define KALMAN_PREDICT_PERIOD_MS 10
#define KALMAN_UPDATE_PERIOD_MS  100

#define USB_PERIOD_MS 250
#define LED_PERIOD_MS 500

#define RAD_TO_DEG 57.2957795131f
#define IMU_INT_Pin 25

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

/* USER CODE BEGIN PV */
uint16_t regAddr = 107;
MPU6050 imu;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t dataCount=0;
int _write(int file, char *ptr, int len) {
	uint8_t status;
	status = CDC_Transmit_FS((uint8_t*) ptr, len);
	return len;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin ){
	/* Check if interrupt pin has fired for IMU */
	if( GPIO_Pin == IMU_INT_Pin ){

	imu.dataReadyFlag = 1;
	dataCount++;
	}
}

/* DMA RECEPTION COMPELTE HANDLER*/
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef * hi2c){

	if ( hi2c->Instance == hi2c1. Instance ) {

	imu.rxFlag = 0;
	MPU6050_Process_Data(&imu);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t w = 10;
	uint8_t read;
	HAL_StatusTypeDef stat;
	KalmanRollPitch ekf;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  //Initialize MPU6050
  MPU6050_Init(&imu,&hi2c1);

  /*Initialize Kalman filter*/
  float KalmanQ[2]={KALMAN_Q,KALMAN_Q};
  float KalmanR[3]={KALMAN_R,KALMAN_R,KALMAN_R};
  KalmanRollPitch_Init(&ekf, KALMAN_P_INIT, KalmanQ, KalmanR);
  /*Initialize timers*/
  uint32_t timerKalmanPredict = 0;
  uint32_t timerKalmanUpdate = 0;

  uint32_t timerUSB = 0;
  uint32_t timerLED = 0;

  float gyrPrev[3]={0.0f,0.0f,0.0f};
  float accPrev[3]={0.0f,0.0f,0.0f};



//  printf("Initialization done");
//  printf("\n starting..");
//  // Go through all possible i2c addresses
	HAL_Delay(3000);
	for (uint8_t(i) = 0; i < 128; i++) {
//
		if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t) (i << 1), 3, 500)== HAL_OK) {
//  		  // We got an ack
			printf("The address is (%d) \n", i);
			HAL_Delay(500);
		} else {
			printf("-- ");
		}
		if (i > 0 && (i + 1) % 16 == 0)
			(printf("\n"));
//
//
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		/* Check if data is ready and DMA transfer is not currently in progress */
		if ( imu.dataReadyFlag && !imu.rxFlag ) {

		/* Filter IMU data */
		imu.gyr_rps[0] = LPF_GYR_ALPHA * gyrPrev[0] + (1.0f - LPF_GYR_ALPHA) * imu.gyr_rps[0];
		imu.gyr_rps[1] = LPF_GYR_ALPHA * gyrPrev[1] + (1.0f - LPF_GYR_ALPHA) * imu.gyr_rps[1];
		imu.gyr_rps[2] = LPF_GYR_ALPHA * gyrPrev[2] + (1.0f - LPF_GYR_ALPHA) * imu.gyr_rps[2];

		imu.acc_mps2[0] = LPF_ACC_ALPHA * accPrev[0] + (1.0f - LPF_ACC_ALPHA) * imu.acc_mps2[0];
		imu.acc_mps2[1] = LPF_ACC_ALPHA * accPrev[1] + (1.0f - LPF_ACC_ALPHA) * imu.acc_mps2[1];
		imu.acc_mps2[2] = LPF_ACC_ALPHA * accPrev[2] + (1.0f - LPF_ACC_ALPHA) * imu.acc_mps2[2];

		/* Initiate new DMA transfer */
		MPU6050_Read_DMA(&imu);
		}

		/* Check if time for Kalman filter prediction step (gyro data available) */
		if ( (HAL_GetTick() - timerKalmanPredict) >= KALMAN_PREDICT_PERIOD_MS ) {

		KalmanRollPitch_Predict(&ekf, imu.gyr_rps, 0.001f *KALMAN_PREDICT_PERIOD_MS);

		timerKalmanPredict += KALMAN_PREDICT_PERIOD_MS;
		}

		/* Check if time for Kalman filter update step (accelerometer data available) */
		if ( (HAL_GetTick() - timerKalmanUpdate) >= KALMAN_UPDATE_PERIOD_MS ) {

		KalmanRollPitch_Update(&ekf, imu.acc_mps2);

		timerKalmanUpdate += KALMAN_UPDATE_PERIOD_MS;
		}

		/* USB data stream */
		if ( (HAL_GetTick() - timerUSB) >= USB_PERIOD_MS ) {
		uint8_t usbBuf[64];
		uint8_t usbBuflen = snprintf(usbBuf, 64, "%.2f,%.2f\r\n", ekf.phi * RAD_TO_DEG, ekf.theta * RAD_TO_DEG);
		CDC_Transmit_FS(usbBuf, usbBuflen);
		timerUSB += USB_PERIOD_MS;
		}

		/* Toggle LED */
		if ( (HAL_GetTick() - timerLED) >= LED_PERIOD_MS ) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		timerLED += LED_PERIOD_MS;
		}

/*
		if(writeByte(208, 107, t5)==true){
			printf("sucessful write to register 107, data 10 written\n");
		}

		if (HAL_I2C_Mem_Read(&hi2c1, 208, 107, 1, &read, 1, 1000) == HAL_OK) {
			printf("(%d). Register 107 of MPU6050 is (%d)\n", i, read);
		}
		//	HAL_Delay(250);
		//writeBits(208, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);
		if (readByte(208, 107, &o_byte, 1000) == 1) {
			i++;
			if (i == 100) {
				i = 0;
			}

		}
		if(HAL_I2C_Mem_Write(&hi2c1, 208, 35, 1, &w_byte, 1, 1000)==HAL_OK){
			printf("FIFO_En buffer enabled]\n");
		}
		if(writeBit(208, 106, 2, 1)==true){
			fif=getFIFOCount();
			printf("WRITE BIT FUNCTION SUCCESSFUL, fifo count %d \n",fif);
		}
		if(HAL_I2C_Mem_Read(&hi2c1, 208, 106, 1, &usr_ctrl, 1, 1000)==HAL_OK){
			printf("Current value of user control register is %d",user_control);
		}
		HAL_Delay(250);
*/


  /* USER CODE END 3 */
}
	}/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 104;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
