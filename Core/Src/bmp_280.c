/*****************************************Custom driver for the BMP280 barometric pressure eHsensor (altitude detection)**********************************/

#include "init.h"
#include "math.h"
#include "stdio.h"
#include "RingBuf.h"
#include "BMP_280.h"
#include "DefFuncs.h"

extern SPI_HandleTypeDef hspi3;

void bmp280_init(bmp280* bmp)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);

	uint8_t temp[2]; 
	temp[0] = REG_WHO_AM_I_BMP | 0x80;
	temp[1] = 0x00;	
	uint8_t WhoAmI[2];
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t *) &temp, (uint8_t *) &WhoAmI, 2, HAL_MAX_DELAY);
	if (WhoAmI[1] == WHO_AM_I_RETURN_BMP)
	{
		sendS("c", "Device identified successfully\n\r");
	} else
	{
		sendS("c", "Device could not be identified\n\r");
	}

	HAL_Delay(1000);
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);

	temp[0] = REG_CTRLMES_BMP & 0x7F;
	temp[1] = CTRLMES;
	HAL_StatusTypeDef ret = HAL_SPI_Transmit(&hspi3, (uint8_t *) &temp, 2, HAL_MAX_DELAY);
	if (ret == HAL_OK)
	{
		sendS("c", "Writing to Control Measurement register\n\r");
	} else
	{
		sendS("c", "Failed writing to Control Measurement register\n\r");
	}

	HAL_Delay(1000);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);

	temp[0] = REG_CONFIG_BMP & 0x7F;
	temp[1] = CONFIG;
	ret = HAL_SPI_Transmit(&hspi3, (uint8_t *) &temp, 2, HAL_MAX_DELAY);
	if (ret == HAL_OK)
	{
		sendS("c", "Writing to Configuration register\n\r");
	} else
	{
		sendS("c", "Failed writing to Configuration register\n\r");
	}

	HAL_Delay(1000);
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);

	uint8_t rawTrimTx[25]; 
	rawTrimTx[0] = REG_TRIMPAR_BMP | 0x80;
	uint8_t rawTrim[25];
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t *) &rawTrimTx, (uint8_t *) &rawTrim, 25, HAL_MAX_DELAY);
	if (ret == HAL_OK)
	{
		sendS("c", "Reading Trimming Parameters\n\r");
	} else 
	{
		sendS("c", "Failed reading Trimming Parameters\n\r");
	} 
	bmp->trimPar[0] = (rawTrim[2] << 8) | rawTrim[1];
        bmp->trimPar[1] = (rawTrim[4] << 8) | rawTrim[3];
        bmp->trimPar[2] = (rawTrim[6] << 8) | rawTrim[5];
        bmp->trimPar[3] = (rawTrim[8] << 8) | rawTrim[7];
	bmp->trimPar[4] = (rawTrim[10] << 8) | rawTrim[9];
        bmp->trimPar[5] = (rawTrim[12] << 8) | rawTrim[11];
	bmp->trimPar[6] = (rawTrim[14] << 8) | rawTrim[13];
	bmp->trimPar[7] = (rawTrim[16] << 8) | rawTrim[15];
	bmp->trimPar[8] = (rawTrim[18] << 8) | rawTrim[17];
	bmp->trimPar[9] = (rawTrim[20] << 8) | rawTrim[19];
	bmp->trimPar[10] = (rawTrim[22] << 8) | rawTrim[21];
	bmp->trimPar[11] = (rawTrim[24] << 8) | rawTrim[23];  

        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);

	sendS("c", "Configuration completed!\n\r");	

}

void bmp280_PresCal(bmp280* bmp, uint16_t it)
{
        sendS("c", "Beginning Altitude Calibration\n\r");
        uint8_t dataPTx[7];
	dataPTx[0] = REG_DATA_PRES | 0x80;
	uint8_t dataPRx[7];
	uint16_t* dig_T1 = (uint16_t *) &(bmp->trimPar[0]);
	int16_t* dig_T2 = &(bmp->trimPar[1]);
	int16_t* dig_T3 = &(bmp->trimPar[2]);
	uint16_t* dig_P1 = (uint16_t *) &(bmp->trimPar[3]);
	int16_t* dig_P2 = &(bmp->trimPar[4]);
	int16_t* dig_P3 = &(bmp->trimPar[5]);
	int16_t* dig_P4 = &(bmp->trimPar[6]);
	int16_t* dig_P5 = &(bmp->trimPar[7]);
	int16_t* dig_P6 = &(bmp->trimPar[8]);
	int16_t* dig_P7 = &(bmp->trimPar[9]);
	int16_t* dig_P8 = &(bmp->trimPar[10]);
	int16_t* dig_P9 = &(bmp->trimPar[11]);

        float pSum = 0;

        for (uint16_t i = 0; i < it; i++)
        {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(&hspi3, (uint8_t *) &dataPTx, (uint8_t *) &dataPRx, 7, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);


	int32_t pres = (dataPRx[1] << 12) | (dataPRx[2] << 4) | (dataPRx[3] >> 4);

	int32_t temp = (dataPRx[4] << 12) | (dataPRx[5] << 4) | (dataPRx[6] >> 4); 


	int64_t var1 = ((((temp>>3) - ((signed long int)*dig_T1<<1))) * ((signed long int)*dig_T2)) >> 11;
	int64_t var2 = (((((temp>>4) - ((signed long int) *dig_T1)) * ((temp>>4) - ((signed long int)*dig_T1))) >> 12) * ((signed long int)*dig_T3)) >> 14;
	int32_t t_fine = var1 + var2;
	bmp->bmpTemp = (t_fine * 5 + 128) >> 8;
	bmp->bmpTemp = bmp->bmpTemp/100.0f;

	var1 = ((int64_t)t_fine) - 128000;
	var2 = (var1) * (var1) * (int64_t)*dig_P6;
	var2 = (var2) + (((var1)*(int64_t)*dig_P5)<<17);
	var2 = (var2) + (((int64_t)*dig_P4)<<35);
	var1 = (((var1) * (var1) * (int64_t)*dig_P3)>>8) + (((var1) * (int64_t)*dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+(var1)))*((int64_t)*dig_P1)>>33;
	int64_t p = 1048576-(pres);
	p = (((p<<31)-(var2))*3125)/(var1);
	var1 = (((int64_t)*dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)*dig_P8) * (p)) >> 19;
	p = ((p + (var1) + (var2)) >> 8) + (((int64_t)*dig_P7)<<4);
	p = (uint32_t)p; 


	bmp->bmpPres = p/(100.0f*256.0f);
	bmp->bmpAlt = ((44330.0f*(1.0f-pow(bmp->bmpPres/1013.25f, 1.0f/5.255f))));

//	sendS("f", bmp->bmpPres);newL();


	pSum += bmp->bmpAlt;
//	sendMati((uint8_t *) &dataPRx, 1, 7);

        HAL_Delay(KALMAN_PREDICT_TIME_ALT);
        }

        bmp->bmpAltCal = pSum/it;
        sendS("c", "Altitude Calibration completed\n\r");
}

void bmp280_PresTempRead(bmp280 *bmp, uint8_t* data)
{
	/* Remove the next line if the sensor is used in alone stand */
	bmp->bmpAltCal = 0;
	uint16_t* dig_T1 = (uint16_t *) &(bmp->trimPar[0]);
	int16_t* dig_T2 = &(bmp->trimPar[1]);
	int16_t* dig_T3 = &(bmp->trimPar[2]);
	uint16_t* dig_P1 = (uint16_t *) &(bmp->trimPar[3]);
	int16_t* dig_P2 = &(bmp->trimPar[4]);
	int16_t* dig_P3 = &(bmp->trimPar[5]);
	int16_t* dig_P4 = &(bmp->trimPar[6]);
	int16_t* dig_P5 = &(bmp->trimPar[7]);
	int16_t* dig_P6 = &(bmp->trimPar[8]);
	int16_t* dig_P7 = &(bmp->trimPar[9]);
	int16_t* dig_P8 = &(bmp->trimPar[10]);
	int16_t* dig_P9 = &(bmp->trimPar[11]);

	int32_t pres = (data[1] << 12) | (data[2] << 4) | (data[3] >> 4);

	int32_t temp = (data[4] << 12) | (data[5] << 4) | (data[6] >> 4); 


	int64_t var1 = ((((temp>>3) - ((signed long int)*dig_T1<<1))) * ((signed long int)*dig_T2)) >> 11;
	int64_t var2 = (((((temp>>4) - ((signed long int) *dig_T1)) * ((temp>>4) - ((signed long int)*dig_T1))) >> 12) * ((signed long int)*dig_T3)) >> 14;
	int32_t t_fine = var1 + var2;
	bmp->bmpTemp = (t_fine * 5 + 128) >> 8;
	bmp->bmpTemp = bmp->bmpTemp/100.0f;

	var1 = ((int64_t)t_fine) - 128000;
	var2 = (var1) * (var1) * (int64_t)*dig_P6;
	var2 = (var2) + (((var1)*(int64_t)*dig_P5)<<17);
	var2 = (var2) + (((int64_t)*dig_P4)<<35);
	var1 = (((var1) * (var1) * (int64_t)*dig_P3)>>8) + (((var1) * (int64_t)*dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+(var1)))*((int64_t)*dig_P1)>>33;
	int64_t p = 1048576-(pres);
	p = (((p<<31)-(var2))*3125)/(var1);
	var1 = (((int64_t)*dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)*dig_P8) * (p)) >> 19;
	p = ((p + (var1) + (var2)) >> 8) + (((int64_t)*dig_P7)<<4);
	p = (uint32_t)p; 


	bmp->bmpPres = p/(100.0f*256.0f);
	bmp->bmpAlt = ((44330.0f*(1.0f-pow(bmp->bmpPres/1013.25f, 1.0f/5.255f))));

	bmp->bmpAlt = bmp->bmpAlt - bmp->bmpAltCal;

}
