/************** This document serves to manage time for the State Machine ************/

#include "Time.h"
#include "DefFuncs.h"
#include "GEKF.h"
#include "AttitudeEstimation.h"
#include "FEKF.h"
#include "StateMachine.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern sensors meas;
extern gekf gkal;
extern fekf fkal;
extern VehicleAtd atd;
extern VehicleStates States;
uint16_t lastTim_T3;
uint16_t lastTim_T4;
uint16_t currTim_T3;
uint16_t currTim_T4;
uint16_t elapsed_T3;
uint16_t elapsed_T4;
float timeSec;
uint8_t startUp_T3 = 1;
uint8_t startUp_T4 = 1;

void delay_us(uint32_t time)
{
	uint32_t initVal = TIM2->CNT;
//	uint32_t initVal = __HAL_TIM_GET_COUNTER(&htim2);

//	uint32_t val = initVal;
	while ((TIM2->CNT - initVal) < time)
	{

//		val = __HAL_TIM_GET_COUNTER(&htim2);
//		val = TIM2->CNT;
	}

//	sendS("i", (val - initVal));space();sendS("i", val);space();sendS("i", initVal);newL();

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		if (startUp_T3)
		{
			lastTim_T3 = __HAL_TIM_GET_COUNTER(&htim2);
			startUp_T3 = 0;
		}
		currTim_T3 = __HAL_TIM_GET_COUNTER(&htim2);
		elapsed_T3 = (currTim_T3 - lastTim_T3);
//		timeSec = (float) elapsed_T3/1000000.0f;
		lastTim_T3 = currTim_T3;
		GroundEKF_newMeas(&gkal, &meas);

		if (States.currentState > GROUND)
		{
//			sendS("i", (int) elapsed_T3);sendS("c", "\n\r");  
//			sendMat((float *) &meas.meas[0], 1, 7);newL();
			FlightEKF_newMeas(&fkal, &meas); 
		}

	}

	if (htim->Instance == TIM4)
	{
		if (startUp_T4)
		{
			startUp_T4 = 0;
		}


		if (States.currentState > GROUND)
		{
			Attitude_newMeas(&atd, &meas);
		}


	}
}
