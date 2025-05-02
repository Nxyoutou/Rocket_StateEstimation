/**************************************************************************************************************/
/****************************This is the file containing important and repetitive functions********************/

#include "stdio.h"
#include "string.h"
#include "stdarg.h"
#include "DefFuncs.h"
#include "inttypes.h"
#include "init.h"

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;


void sendS(const char* str, ...)
{
        va_list args;
        va_start(args, str);
        if (*str == 'c')
        {
                const char* arg = va_arg(args, const char*);
                uint8_t size = strlen((const char*) arg);
                HAL_UART_Transmit(&huart4, (uint8_t *) arg, size, HAL_MAX_DELAY);
        } else if ((*str == 'f') | (*str == 'd'))
        {
                float arg = va_arg(args, double);
                const char* TxBuf[16];
                sprintf((char *) TxBuf, "%f", arg);
                uint8_t size = strlen((const char *) TxBuf);
                HAL_UART_Transmit(&huart4, (uint8_t *) TxBuf, size, HAL_MAX_DELAY);
	} else if (*str == 'i')
	{
		int arg = va_arg(args, int);
		const char* TxBuf[16];
		sprintf((char *) TxBuf, "%d", arg);
		uint8_t size = strlen((const char *) TxBuf);
		HAL_UART_Transmit(&huart4, (uint8_t *) TxBuf, size, HAL_MAX_DELAY);
        } else
        {
                const char* TxBuf[16];
                sprintf((char *) TxBuf, "Invalid data type\n\r");
                HAL_UART_Transmit(&huart4, (uint8_t *) TxBuf, strlen((const char*) TxBuf), HAL_MAX_DELAY);
        }
        va_end(args);

}

void sendMat(float* data, uint8_t r, uint8_t c)
{
	for (int i = 0; i < (r*c); i++)
	{
		((i % c) == 0)? (sendS("c", "[")): (sendS("c", "\0"));

		sendS("c", "   ");
		sendS("f", data[i]);
		
		(((i+1) % c) == 0)? sendS("c", "]\n\r"): (sendS("c", "\0"));
	}
}

void sendMati(uint8_t* data, uint8_t r, uint8_t c)
{
	for (int i = 0; i < (r*c); i++)
	{
		((i % c) == 0)? (sendS("c", "[")): (sendS("c", "\0"));

		sendS("c", "   ");
		sendS("i", data[i]);
		
		(((i+1) % c) == 0)? sendS("c", "]\n\r"): (sendS("c", "\0"));
	}


}


void sendMatc(char* data, uint8_t r, uint8_t c)
{
	for (int i = 0; i < (r*c); i++)
	{
		((i % c) == 0)? (sendS("c", "[")): (sendS("c", "\0"));

		sendS("c", "   ");
		sendS("c", data[i]);
		
		(((i+1) % c) == 0)? sendS("c", "]\n\r"): (sendS("c", "\0"));
	}


}

void receiveS_B(void* data, uint8_t size)
{
	HAL_UART_Receive(&huart1, (uint8_t *) data, size, HAL_MAX_DELAY);
}

void receiveS_IT(void* data, uint8_t size)
{
	HAL_UART_Receive_IT(&huart1, (uint8_t *) data, size);
}


void space(void)
{
	sendS("c", "     ");
}

void newL(void)
{
	sendS("c", "\n\r");
}

uint8_t APRS_Transmit(uint8_t* data)
{
	HAL_StatusTypeDef ret =	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ((0x09 << 1)), (uint8_t *) data, sizeof(float)*8, HAL_MAX_DELAY);
	if (ret == HAL_OK)
	{
		return 1;
	} else {
		return 0;
	};
}

uint8_t XBee_Transmit(char* data)
{
                uint8_t size = strlen((const char*) data);
               	HAL_UART_Transmit(&huart1, (uint8_t *) data, size, HAL_MAX_DELAY);
		return size;	
}
