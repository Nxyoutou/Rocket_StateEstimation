#ifndef INC_RINGBUF_H_
#define INC_RINGBUF_H_

#include "stdio.h"

#define RingBufSize 64
typedef struct {

	/* Ring buffer size */
	uint32_t size;

        /* Rx Buffer */
	uint8_t* Buf;

	/* Tail position */
	int32_t tail;

	/* Head position */
	uint32_t head;

	/* Measuring the data loss in %*/
	float currentLoss;
	int failed; /* This is not in percent */
	float totalLoss;

	/* Total data transfer */
	int totalTransfer;

	/* Checking whether the Head overlapped the Tail */
	uint16_t overlap;

	/* First time check */
	uint8_t First;

} ring;

uint8_t ring_init(ring* buf, void* data_buf, uint32_t size);
uint32_t ring_DataFullSize(ring* buf);
uint32_t ring_DataEmptySize(ring* buf);
uint8_t ring_IsDataAvailable(ring* buf);
uint32_t ring_WriteData(ring* buf, void* data_buf, uint32_t len, uint8_t overwrite);
uint32_t ring_ReadData(ring* buf, void* data_buf);
float ring_ReadSensorData(ring* buf, uint16_t size);

#endif /* INC_RINGBUF_H_ */
