#ifndef INC_DEFFUNCS_H_
#define INC_DEFFUNCS_H_

#include "stdio.h"


/* Function definitions */
void sendS(const char* str, ...);
void sendMat(float* data, uint8_t r, uint8_t c);
void sendMati(uint8_t* data, uint8_t r, uint8_t c);
void sendMatc(char* data, uint8_t r, uint8_t c);
void receiveS_B(void* data, uint8_t size);
void receiveS_IT(void* data, uint8_t size);
uint8_t APRS_Transmit(uint8_t* data);
uint8_t XBee_Transmit(char* data);
void space(void);
void newL(void);
























#endif /* INC_DEFFUNCS_H_ */
