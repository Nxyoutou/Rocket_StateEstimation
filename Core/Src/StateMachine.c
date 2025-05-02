#include "init.h"
#include "DefFuncs.h"
#include "StateMachine.h"
#include "Sensors.h"
#include "BMP_280.h"
#include "Time.h"
#include "GEKF.h"
#include "FEKF.h"
#include "AttitudeEstimation.h"
#include "States.h"
#include "fatfs.h"
#include "fatfs_sd.h"

sensors meas;
extern bmp280 bmp;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern uint8_t finalData[8];
extern uint8_t stat;

FATFS fs;
FIL fil;
DIR dir;
FRESULT fresult;
char buffer[1024];

UINT br, bw;

FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;



sensors meas;
gekf gkal;
fekf fkal;
VehicleAtd atd;
VehicleStates States;

uint32_t generalTime;
uint32_t generalTimeStart;
uint8_t startUpMeas = 1;
uint8_t notLanded = 1;
uint8_t statXBee;


void statemachine_init(void)
{
        init_measurements(&meas);
        GroundEKF_init(&gkal, 0.01f, 1.0f);
        Attitude_init(&atd);
        FlightEKF_init(&fkal, 0.01f, 0.003f);
        States.currentState = GROUND;
        States.StateHandlers[BASE] = BASE_Handle;
        States.StateHandlers[GROUND] = GROUND_Handle;
        States.StateHandlers[ARMED] = ARMED_Handle;
        States.StateHandlers[FASTASCENT] = FASTASCENT_Handle;
        States.StateHandlers[SLOWASCENT] = SLOWASCENT_Handle;
        States.StateHandlers[APOGEE] = APOGEE_Handle;
        States.StateHandlers[FREEFALL] = FREEFALL_Handle;
        States.StateHandlers[LANDED] = LANDED_Handle;
        HAL_TIM_Base_Start(&htim2);
        HAL_TIM_Base_Start_IT(&htim3);
        HAL_TIM_Base_Start_IT(&htim4);
	fresult = f_mount(&fs, "", 1);

	if (fresult != FR_OK) 
	{
		sendS("c", "Unsuccessful again\r\n");
	} else 
	{
		sendS("c", "Mounted successfully! \r\n");
	}
	
        generalTimeStart = HAL_GetTick();
//	bmp280_PresCal(&bmp, 100);

}

void statemachine_run(void)
{
	if (startUpMeas < 63)
	{
		for (startUpMeas; startUpMeas < 64; startUpMeas++)
		{
			make_measurements(&meas);
		}
	}
	make_measurements(&meas);
//	sendS("f", meas.meas[6]);space();sendS("f", meas.meas[2]);newL();
        States.StateHandlers[States.currentState]();
	generalTime = HAL_GetTick();
//	sendMat(&meas.meas[0], 1, 7);newL();
//	sendS("i", generalTime);newL();


}

void BASE_Handle(void)
{
        BaseState(&States);
}

void GROUND_Handle(void)
{
        GroundState(&gkal, &meas, &States);
}


void ARMED_Handle(void)
{
        ArmedState(&fkal, &atd, &States);
}

void FASTASCENT_Handle(void)
{
        FastAscentState(&fkal, &atd, &States);
}

void SLOWASCENT_Handle(void)
{
        SlowAscentState(&fkal, &atd, &States);
}

void APOGEE_Handle(void)
{
        ApogeeState(&fkal, &atd, &States);
}

void FREEFALL_Handle(void)
{
        FreefallState(&fkal, &atd, &States);
}

void LANDED_Handle(void)
{
        LandedState(&fkal, &atd, &States);
}

