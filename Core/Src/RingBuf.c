/*********************************************Functions enabling the usage of ring buffers************************************************/

#include "init.h"
#include "stdio.h"
#include "string.h"
#include "RingBuf.h"
#include "DefFuncs.h"
#include "math.h"

#define SMALLER(a, b) ((a) <= (b) ? (a) : (b))

uint8_t ring_init(ring* buf, void* data_buf, uint32_t size)
{
	sendS("c", "Configuring Ring Buffer\n\r");
	buf->size = size;
	buf->Buf = (uint8_t *) data_buf;
	buf->head = 1;
	buf->tail = 0;
	buf->currentLoss = 0;
	buf->totalLoss = 0;
	buf->totalTransfer = 0;
	buf->failed = 0;
	buf->First = 1;
	sendS("c", "Finished configuring Ring Buffer\n\r");
	return 1;
}

uint32_t ring_DataFullSize(ring* buf)
{
	uint32_t size;

	if ((buf->head - 1) > buf->tail)
	{
		size = buf->head - 1 - buf->tail;
	} else if ((buf->head - 1) < buf->tail) 
	{
		size = buf->size - 1 - buf->tail + buf->head;
	} else if (((buf->head - 1) == buf->tail) && (buf->overlap > 0))
	{
		size = buf->size;
	} else
	{
		size = 0;
	}
	return size;
}

uint32_t ring_DataEmptySize(ring* buf)
{
	uint32_t size = ring_DataFullSize(buf);
	return buf->size - size;
}


uint8_t ring_IsDataAvailable(ring* buf)
{
	uint32_t size = ring_DataFullSize(buf);
	if (size == 0)
	{
		return 0;
	} else {
		return 1;
	}
}

	
uint32_t ring_WriteData(ring* buf, void* data_buf, uint32_t len, uint8_t overwrite)
{

	uint32_t toTransfer;
	uint8_t* data = (uint8_t *) data_buf;

	uint32_t free = ring_DataEmptySize(buf);

	if (free < len)
	{
		if (!overwrite)
		{
			return 0;
		}

		buf->failed += (len - free);
		buf->currentLoss = (len - free)/len;
		buf->totalLoss = buf->failed/(buf->totalTransfer + len);
		buf->overlap += 1;
	}

	buf->totalTransfer += len;

	uint32_t rest = buf->size - buf->head;

	toTransfer = SMALLER(rest, len);
	memcpy(&buf->Buf[buf->head], data, toTransfer);
	buf->head += toTransfer;
	data += toTransfer;
	len -= toTransfer;

	if (len > 0)
	{
		memcpy(buf->Buf, data, len);
		buf->head = len;
	}

	if ((buf->head >= buf->size))
	{
		buf->head = 0;
	}

	return toTransfer+len;
}

uint32_t ring_ReadData(ring* buf, void* data_buf)
{
	
	uint8_t* data = (uint8_t *) data_buf;

	uint32_t size = ring_DataFullSize(buf);

	if ((buf->tail >= (buf->size - 1)))
	{
		buf->tail = -1;
	}

	uint32_t rest = buf->size - buf->tail - 1;
	uint32_t toTransfer = SMALLER(rest, size);
	memcpy(data, &buf->Buf[buf->tail + 1], toTransfer);
	buf->tail += toTransfer;
	data += toTransfer;
	size -= toTransfer;

	if (size > 0)
	{
		memcpy(data, buf->Buf, size);
		buf->tail = size - 1;
	}


	return toTransfer + size;
}

/* Only call in combination with ring_IsDataAvailable or if you are sure that there will be data in the buffer when this function is called*/
float ring_ReadSensorData(ring* buf, uint16_t size)
{
     float sum = 0.0f;

     uint8_t* x = (uint8_t *) buf->Buf + 1;

//     float* data = (float *) x;
     uint8_t arr[4];
     float* variable;
     variable = (float *)&arr[0];
     size = (size == (buf->size/4))? (size-1) : size;

//     uint32_t initVal = TIM2->CNT;

     for (int i = 0; i < (size*4); i+=4)
     {
	     arr[0] = x[i];
	     arr[1] = x[i+1];
	     arr[2] = x[i+2];
	     arr[3] = x[i+3];

    	     sum += variable[0];
     }

//     uint32_t endVal = TIM2->CNT;

//     sendS("i", (endVal-initVal));newL();

	
//     sendS("i", x[0]);space();sendS("i", x[1]);space();sendS("i", x[2]);space();sendS("i", x[3]);space();sendS("f", variable[0]);newL();
     return sum/((float) size);
}
