/********************************************** This file contains the Sensor functions for the IMU and the Barometer **************************************/

#include "init.h"
#include "ICM_20649.h"
#include "BMP_280.h"
#include "Sensors.h"
#include "RingBuf.h"
#include "DefFuncs.h"
#include "string.h"


extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi1;
extern sensors meas;
icm icm20649;
bmp280 bmp;


__attribute__((section(".buffer"))) uint8_t dataIMUTx[13];
__attribute__((section(".buffer"))) uint8_t dataIMURx[13];
__attribute__((section(".buffer"))) uint8_t dataBMPTx[7];
__attribute__((section(".buffer"))) uint8_t dataBMPRx[7];



void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
        if (hspi->Instance == SPI1)
        {
	
                meas.flagIMU = 1;
		HAL_SPI_DMAStop(&hspi1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);


		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

		SCB_InvalidateDCache_by_Addr((uint32_t*)(((uint32_t)dataIMURx) & ~(uint32_t)0x1F), sizeof(dataIMURx)+32);

		HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *) &dataIMUTx, (uint8_t *) &dataIMURx, sizeof(dataIMURx));


	} else if (hspi->Instance == SPI3) 
	{

	         meas.flagBMP = 1;
                 HAL_SPI_DMAStop(&hspi3);
                 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);


                 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);

		 SCB_InvalidateDCache_by_Addr((uint32_t*)(((uint32_t)dataBMPRx) & ~(uint32_t)0x1F), sizeof(dataBMPRx)+32);
	
                 HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t *) &dataBMPTx, (uint8_t *) &dataBMPRx, sizeof(dataBMPRx));


	}
}




ring buf_IMUAccX;
ring buf_IMUAccY;
ring buf_IMUAccZ;
ring buf_IMUGyrX;
ring buf_IMUGyrY;
ring buf_IMUGyrZ;

ring buf_AltZ;
ring buf_Temp;

float IMUAccX[64];
float IMUAccY[64];
float IMUAccZ[64];
float IMUGyrX[64];
float IMUGyrY[64];
float IMUGyrZ[64];

float AltZ[64];
float Temp[32];

float x_acc;
float y_acc;
float z_acc;
float x_gyr;
float y_gyr;
float z_gyr;

float z_alt;
float temp;

void init_measurements(sensors* meas)
{

	for (int j = 0; j < 64; j++)
	{
		AltZ[j] = 0.0f;
		IMUGyrX[j] = 0.0f;
		IMUGyrY[j] = 0.0f;
		IMUGyrZ[j] = 0.0f;
		IMUAccX[j] = 0.0f;
		IMUAccY[j] = 0.0f;
		IMUAccZ[j] = 0.0f;
		if (j < 32)
		{
			Temp[j] = 0.0f;
		}
	}

  
	for (int z = 0; z < 13; z++)

     	{
      		dataIMUTx[z] = 0;
		dataIMURx[z] = 0;
		if (z < 7)
		{
			dataBMPTx[z] = 0;
			dataBMPRx[z] = 0;
		}
      	}

	icm20649_init(&icm20649);
	HAL_Delay(1000);
	bmp280_init(&bmp);
	HAL_Delay(1000);
	ring_init(&buf_IMUAccX, &IMUAccX, sizeof(IMUAccX));
	HAL_Delay(500);
	ring_init(&buf_IMUAccY, &IMUAccY, sizeof(IMUAccY));
	HAL_Delay(500);
	ring_init(&buf_IMUAccZ, &IMUAccZ, sizeof(IMUAccZ));
	HAL_Delay(500);
	ring_init(&buf_IMUGyrX, &IMUGyrX, sizeof(IMUGyrX));
	HAL_Delay(500);
	ring_init(&buf_IMUGyrY, &IMUGyrY, sizeof(IMUGyrY));
	HAL_Delay(500);
	ring_init(&buf_IMUGyrZ, &IMUGyrZ, sizeof(IMUGyrZ));
	HAL_Delay(500);
	ring_init(&buf_AltZ, &AltZ, sizeof(AltZ));
	HAL_Delay(500);
	ring_init(&buf_Temp, &Temp, sizeof(Temp));
	dataBMPTx[0] = REG_DATA_PRES | 0x80;
	dataIMUTx[0] = REG_ACC_DATA | 0x80;


	meas->flagIMU = 0;
	meas->flagBMP = 0;
	meas->FirstDMA = 1;
	icm20649.allowTrigger = 0;
}

void make_measurements(sensors* meas)
{

	if (meas->FirstDMA)
	{

		user_bank(0);	

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

		SCB_CleanDCache_by_Addr((uint32_t*)(((uint32_t)dataIMUTx) & ~(uint32_t)0x1F), sizeof(dataIMUTx)+32);

		SCB_InvalidateDCache_by_Addr((uint32_t*)(((uint32_t)dataIMURx) & ~(uint32_t)0x1F), sizeof(dataIMURx)+32);

	        HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *) &dataIMUTx, (uint8_t *) &dataIMURx, sizeof(dataIMURx));


  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

		SCB_CleanDCache_by_Addr((uint32_t*)(((uint32_t)dataBMPTx) & ~(uint32_t)0x1F), sizeof(dataBMPTx)+32);

		SCB_InvalidateDCache_by_Addr((uint32_t*)(((uint32_t)dataBMPRx) & ~(uint32_t)0x1F), sizeof(dataBMPRx)+32);

  		HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t *) &dataBMPTx, (uint8_t *) &dataBMPRx, sizeof(dataBMPRx)); 


		meas->FirstDMA = 0;
	}

//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

//	  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *) &dataIMUTx, (uint8_t *) &dataIMURx, sizeof(dataIMURx), HAL_MAX_DELAY);

//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

	if (meas->flagIMU)
	{

		icm20649_DataRead(&icm20649, (uint8_t *) &dataIMURx);

                ring_WriteData(&buf_IMUAccX, &icm20649.AccM[0], sizeof(icm20649.AccM[0]), 1);
                ring_WriteData(&buf_IMUAccY, &icm20649.AccM[1], sizeof(icm20649.AccM[1]), 1);
                ring_WriteData(&buf_IMUAccZ, &icm20649.AccM[2], sizeof(icm20649.AccM[2]), 1);
		ring_WriteData(&buf_IMUGyrX, &icm20649.GyroM[0], sizeof(icm20649.GyroM[0]), 1);
                ring_WriteData(&buf_IMUGyrY, &icm20649.GyroM[1], sizeof(icm20649.GyroM[1]), 1);
                ring_WriteData(&buf_IMUGyrZ, &icm20649.GyroM[2], sizeof(icm20649.GyroM[2]), 1);

		x_acc = ring_ReadSensorData(&buf_IMUAccX, 64); 
          	y_acc = ring_ReadSensorData(&buf_IMUAccY, 64); 
          	z_acc = ring_ReadSensorData(&buf_IMUAccZ, 64); 
		x_gyr = ring_ReadSensorData(&buf_IMUGyrX, 64); 
          	y_gyr = ring_ReadSensorData(&buf_IMUGyrY, 64); 
          	z_gyr = ring_ReadSensorData(&buf_IMUGyrZ, 64); 


		meas->meas[0] = x_acc;
		meas->meas[1] = y_acc;
		meas->meas[2] = z_acc;

		meas->meas[3] = x_gyr;
		meas->meas[4] = y_gyr;
		meas->meas[5] = z_gyr;


		meas->flagIMU = 0;


//		sendS("f", x_acc);sendS("c", "     ");sendS("f", y_acc);sendS("c", "     ");sendS("f", z_acc);sendS("c", "\n\r");
//		sendS("f", icm20649.AccM[0]);sendS("c", "     ");sendS("f", icm20649.AccM[1]);sendS("c", "     ");sendS("f", icm20649.AccM[2]);sendS("c", "\n\r");


        }

	if (meas->flagBMP)
	{
		bmp280_PresTempRead(&bmp, (uint8_t *) &dataBMPRx);

                ring_WriteData(&buf_AltZ, &bmp.bmpAlt, sizeof(bmp.bmpPres), 1);
		ring_WriteData(&buf_Temp, &bmp.bmpTemp, sizeof(bmp.bmpTemp), 1);

		z_alt = ring_ReadSensorData(&buf_AltZ, 64);
		temp = ring_ReadSensorData(&buf_Temp, 16);
//		sendS("f", z_alt);newL();


		meas->meas[6] = z_alt;
		meas->meas[7] = temp;

                meas->flagBMP = 0;


	}
}
