/************** This is the document to define all the Vehicle State functions **************/
#include "init.h"
#include "States.h"
#include "DefFuncs.h"

char dataPol[4];
char dataIT[2];
float finalData[8] = {1.0f, 2.2f, 3.0f, 4.57f, 5.37f, 6.24f, 7.30f, 0.00f};
uint8_t print_Arm_msg = 1;

uint8_t stat;

extern uint32_t generalTime;
extern VehicleStates States;
extern sensors meas;

uint32_t GroundStart;
uint32_t ArmStart;
uint32_t FastAscentStart;
uint32_t SlowAscentStart;
uint32_t ApogeeStart;
uint32_t FreefallStart;
uint32_t LandedStart;


uint8_t GroundFirst = 1;
uint8_t ArmFirst = 1;
uint8_t FastAscentFirst = 1;
uint8_t SlowAscentFirst = 1;
uint8_t ApogeeFirst = 1;
uint8_t FreefallFirst = 1;
uint8_t LandedFirst = 1;



void BaseState(VehicleStates* State)
{
	if (print_Arm_msg)
	{

		sendS("c", "Entered Base State, to arm the vehicle type 'ARM'...\n\r");
		print_Arm_msg = 0;
	}

	receiveS_B(dataPol, 3);
	if (strcmp(dataPol, "ARM") == 0)
	{
		sendS("c", "Transitoning into GROUND State...\n\r");
		State->currentState = GROUND;
	}
	receiveS_IT(dataIT, sizeof(dataIT));

}


void GroundState(gekf* gkal, sensors* meas, VehicleStates* State)
{
	if (ArmFirst)
	{
		ArmStart = HAL_GetTick();
		ArmFirst = 0;
		receiveS_IT(dataIT, sizeof(dataIT));
		sendS("c", "Enabled interrupt...\n\r");
	}

	GroundEKF_run(gkal, meas);
	if (gkal->armed == 1)
	{
		sendS("c", "Rocket is now ARMED...\n\r");
		State->currentState = ARMED;
	}
}

void ArmedState(fekf* fkal, VehicleAtd* atd, VehicleStates* State)
{
	if (GroundFirst)
	{
		GroundStart = HAL_GetTick();
		GroundFirst = 0;
		receiveS_IT(dataIT, sizeof(dataIT));
		sendS("c", "Enabled interrupt...\n\r");

	}

	Attitude_run(atd);
	FlightEKF_run(fkal, atd);
	if (fkal->x.pData[5] > 2)
	{
		sendS("c", "Rocket in FAST ASCENT...\n\r");
		XBee_Transmit("Rocket in FAST ASCENT...\n\r");
		State->currentState = FASTASCENT;
	}

}

void FastAscentState(fekf* fkal, VehicleAtd* atd, VehicleStates* State)
{
	if (FastAscentFirst)
	{
		FastAscentStart = HAL_GetTick();
		FastAscentFirst = 0;
		receiveS_IT(dataIT, sizeof(dataIT));

	}

	Attitude_run(atd);
	FlightEKF_run(fkal, atd);

	if ((HAL_GetTick() - FastAscentStart) > BURN_TIME)
	{
		sendS("c", "Rocket in SLOW ASCENT...\n\r");
		XBee_Transmit("Rocket in SLOW ASCENT...\n\r");
		State->currentState = SLOWASCENT;
	}

}

void SlowAscentState(fekf* fkal, VehicleAtd* atd, VehicleStates* State)
{
	if (SlowAscentFirst)
	{
		SlowAscentStart = HAL_GetTick();
		SlowAscentFirst = 0;
		receiveS_IT(&dataIT, sizeof(dataIT));

	}

	Attitude_run(atd);
	FlightEKF_run(fkal, atd);

	if ((fkal->x.pData[5] > (-1.0f)) && (fkal->x.pData[5] < 1.0f))
	{
		sendS("c", "APOGEE detected...\n\r");
		XBee_Transmit("APOGEE detected...\n\r");
		State->currentState = APOGEE;
	}

}

void ApogeeState(fekf* fkal, VehicleAtd* atd, VehicleStates* State)
{
	if (ApogeeFirst)
	{
		ApogeeStart = HAL_GetTick();
		ApogeeFirst = 0;
		receiveS_IT(&dataIT, sizeof(dataIT));

	}

	Attitude_run(atd);
	FlightEKF_run(fkal, atd);

	if ((fkal->x.pData[5] < (-1.0f)) )
	{
		sendS("c", "Rocket in FREEFALL...\n\r");
		XBee_Transmit("Rocket in FREEFALL...\n\r");
		State->currentState = FREEFALL;
	}

}

void FreefallState(fekf* fkal, VehicleAtd* atd, VehicleStates* State)
{
	if (FreefallFirst)
	{
		FreefallStart = HAL_GetTick();
		FreefallFirst = 0;
		receiveS_IT(&dataIT, sizeof(dataIT));
	}

	Attitude_run(atd);
	FlightEKF_run(fkal, atd);

	if ((fkal->x.pData[5] > (-1.0f)) && (fkal->x.pData[5] < 1.0f))
	{
		sendS("c", "Rocket LANDED...\n\r");
		States.APRS[2] = fkal->x.pData[5];
		XBee_Transmit("Rocket LANDED...\n\r");
		State->currentState = LANDED;
	}

}

void LandedState(fekf* fkal, VehicleAtd* atd, VehicleStates* State)
{
	if (LandedFirst)
	{
		LandedStart = HAL_GetTick();
		States.APRS[3] = (float) LandedStart;
		States.APRS[4] = atd->euler[0];
		States.APRS[5] = atd->euler[1];
		States.APRS[7] = meas.meas[7];
		LandedFirst = 0;
		receiveS_IT(&dataIT, sizeof(dataIT));
		sendS("c", "Rocket touch down...\n\r");
	}

	stat =	APRS_Transmit((uint8_t *)States.APRS);
	if (stat == 1)
	{
		sendS("c", "Transmissitting to NASA...\n\r");
	} else {
		sendS("c", "Failed to initiate Transmission\n\r");
	}
	HAL_Delay(1000);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
			sendS("c", "Interrupted...\n\r");

		if (strcmp(dataIT, "TX") == 0)
		{
			XBee_Transmit("Transmit interrupt triggered...\n\r");
			sendS("c", "Interrupted..\n\r");
			States.currentState = LANDED;

		}	
		
		

	}
}
