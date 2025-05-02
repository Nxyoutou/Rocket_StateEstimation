#include "ICM_20649.h"
#include "init.h"
#include "stdio.h"
#include "string.h"
#include "DefFuncs.h"

extern SPI_HandleTypeDef hspi1;

void user_bank(uint8_t USR_BANK)
{
	uint8_t tempUSR[2];

	tempUSR[0] = REG_USR_BANK & 0x7F;

	switch (USR_BANK) {
		case 0:
			tempUSR[1] = USR_BANK_0;
			break;
		case 1:
			tempUSR[1] = USR_BANK_1;
			break;
		case 2:
			tempUSR[1] = USR_BANK_2;
			break;
		case 3:
			tempUSR[1] = USR_BANK_3;
			break;

		default:
			sendS("c", "INVALID USER BANK\n\r");

	}

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi1, (uint8_t*) &tempUSR, 2, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

}


void icm20649_init(icm* kal)
{
         uint8_t temp[2];
         uint8_t WhoAmI[2];
         uint8_t USER_CTRL[2];


         user_bank(0);

         temp[0] = REG_PWR_MGMT & 0x7F;
         temp[1] = PWR_MGMT_INIT;

          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

          HAL_StatusTypeDef ret = HAL_SPI_Transmit(&hspi1, (uint8_t*) temp, 2, HAL_MAX_DELAY);

          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

          if (ret == HAL_OK)
          {

                  sendS("c","Resetting Sensor\n\r");
                  HAL_Delay(1000);

          } else {

                  sendS("c", "FAILED resetting Sensor\n\r");
                  HAL_Delay(1000);

          }

          temp[0] = REG_PWR_MGMT & 0x7F;
          temp[1] = PWR_MGMT_NORM;

          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

          ret = HAL_SPI_Transmit(&hspi1, (uint8_t*) temp, 2, HAL_MAX_DELAY);

          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

          if (ret == HAL_OK)
          {

                  sendS("c","Waking up Sensor\n\r");
                  HAL_Delay(1000);

          } else {

                  sendS("c", "FAILED waking up sensor\n\r");
                  HAL_Delay(1000);

          }


          temp[0] = REG_WHO_AM_I_ICM | 0x80;
          temp[1] = 0x00;

          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

          /* Check if device is ready */
          HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) temp, (uint8_t*) WhoAmI, 2, HAL_MAX_DELAY);

          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);


          if (WhoAmI[1] == WHO_AM_I_RETURN_ICM)
          {
                  sendS("c","Device is Ready\n\r");
                  HAL_Delay(1000);
          } else {
                  sendS("c", "Device is NOT Ready\n\r");
                  sendS("i", WhoAmI[1]);newL();
                  HAL_Delay(1000);
          }


          temp[0] = REG_USER_CTRL | 0x80;
          temp[1] = 0x00;


          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

          ret = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) temp, (uint8_t *) USER_CTRL, 2, HAL_MAX_DELAY);

          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

          temp[0] = REG_USER_CTRL & 0x7F;
          temp[1] = USER_CTRL[1];
          temp[1] |= 0x10;

          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

          ret = HAL_SPI_Transmit(&hspi1, (uint8_t*) temp, 2, HAL_MAX_DELAY);

          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);



          if (ret == HAL_OK)
          {

                  sendS("c","Writing to Control User Register\n\r");
                  HAL_Delay(1000);

          } else {

                  sendS("c", "FAILED writing to Control User Register\n\r");
                  HAL_Delay(1000);

          }
          user_bank(2);


           temp[0] = REG_GYR_SMPLRT & 0x7F;
           temp[1] = GYR_SMPLRT_DIV;

           HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

           ret = HAL_SPI_Transmit(&hspi1, (uint8_t*) temp, 2, HAL_MAX_DELAY);

           HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

           if (ret == HAL_OK)
           {

                   sendS("c","Writing to Gyro Sample Rate Register\n\r");
                   HAL_Delay(1000);

           } else {

                   sendS("c", "FAILED writing to Gyro Sample Rate Register\n\r");
                   HAL_Delay(1000);

           }


           temp[0] = REG_GYR_CONFIG_I & 0x7F;
           temp[1] = GYR_CONFIG_I;

           HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

           ret = HAL_SPI_Transmit(&hspi1, (uint8_t*) temp, 2, HAL_MAX_DELAY);

           HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

           if (ret == HAL_OK)
           {

                   sendS("c","Writing to Gyro Configuration I Register\n\r");
                   HAL_Delay(1000);

           } else {

                   sendS("c", "FAILED writing to Gyro Configuration I Register\n\r");
                   HAL_Delay(1000);
           }


            temp[0] = REG_GYR_CONFIG_II & 0x7F;
            temp[1] = GYR_CONFIG_II;

            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

            ret = HAL_SPI_Transmit(&hspi1, (uint8_t*) temp, 2, HAL_MAX_DELAY);

            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

            if (ret == HAL_OK)
            {

                    sendS("c","Writing to Gyro Configuration II Register\n\r");
                    HAL_Delay(1000);

            } else {

                    sendS("c", "FAILED writing to Gyro Configuration II Register\n\r");
                    HAL_Delay(1000);

            }


            temp[0] = REG_ACC_SMPLRT_I & 0x7F;
            temp[1] = ACC_SMPLRT_DIV_I;

            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

        //    ret = HAL_SPI_Transmit(&hspi1, (uint8_t*) temp, 2, HAL_MAX_DELAY);

            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

            if (ret == HAL_OK)
            {

                    sendS("c","Writing to Acc Sample Output Rate I Register\n\r");
                    HAL_Delay(1000);

            } else {

                    sendS("c", "FAILED writing to Sample Output Rate I Register\n\r");
                    HAL_Delay(1000);

            }


            temp[0] = REG_ACC_SMPLRT_II & 0x7F;
            temp[1] = ACC_SMPLRT_DIV_II;

            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

    //        ret = HAL_SPI_Transmit(&hspi1, (uint8_t*) temp, 2, HAL_MAX_DELAY);

            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

            if (ret == HAL_OK)
            {

                    sendS("c","Writing to Acc Sample Output Rate II Register\n\r");
                    HAL_Delay(1000);

            } else {

                    sendS("c", "FAILED writing to Sample Output Rate II Register\n\r");
                    HAL_Delay(1000);
            }


             temp[0] = REG_ACC_CONFIG_I & 0x7F;
             temp[1] = ACC_CONFIG_I;

             HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

             ret = HAL_SPI_Transmit(&hspi1, (uint8_t*) temp, 2, HAL_MAX_DELAY);

             HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

             if (ret == HAL_OK)
             {

                     sendS("c","Writing to Acc Configuration I Register\n\r");
                     HAL_Delay(1000);

             } else {

                     sendS("c", "FAILED writing to Acc Configuration I Register\n\r");
                     HAL_Delay(1000);

             }


             temp[0] = REG_ACC_CONFIG_II & 0x7F;
             temp[1] = ACC_CONFIG_II;

             HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

             ret = HAL_SPI_Transmit(&hspi1, (uint8_t*) temp, 2, HAL_MAX_DELAY);

             HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

             if (ret == HAL_OK)
             {

                     sendS("c","Writing to Acc Configuration II Register\n\r");
                     HAL_Delay(1000);

             } else {

                     sendS("c", "FAILED writing to Acc Configuration II Register\n\r");
                     HAL_Delay(1000);

             }


            temp[0] = REG_ODR_ALIGN & 0x7F;
            temp[1] = ODR_ALIGN_EN;

            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

            ret = HAL_SPI_Transmit(&hspi1, (uint8_t*) temp, 2, HAL_MAX_DELAY);

            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

            if (ret == HAL_OK)
            {

                    sendS("c","Writing to Data Output Register\n\r");
                    HAL_Delay(1000);

            } else {

                    sendS("c", "FAILED writing to Output Register\n\r");
                    HAL_Delay(1000);

            }




              user_bank(0);

              sendS("c", "Finished Gyro Configuration!\n\r");

}


void icm20649_DataRead(icm* kal, uint8_t* data)
{
	int16_t x_gyr;
	int16_t y_gyr;
	int16_t z_gyr;

        int16_t x_acc;
        int16_t y_acc;
        int16_t z_acc;

	x_gyr = (data[9] << 8) | data[10];
	y_gyr = (data[7] << 8) | data[8];
	z_gyr = (data[11] << 8) | data[12];

        x_acc = (data[3] << 8) | data[4];
        y_acc = (data[1] << 8) | data[2];
        z_acc = (data[5] << 8) | data[6];

        /* Remove the 9.81 if you want polished data with 1.0 max acceleration */
        kal->GyroM[0] = 1.0f*x_gyr/(65.5f);
        kal->GyroM[1] = 1.0f*y_gyr/(65.5f);
        kal->GyroM[2] = 1.0f*z_gyr/(65.5f);

        kal->AccM[0] = -9.81f*x_acc/(1024.0f);
        kal->AccM[1] = -9.81f*y_acc/(1024.0f);
        kal->AccM[2] = -9.81f*z_acc/(1024.0f);


}
