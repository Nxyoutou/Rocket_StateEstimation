/************************** This document contains the Flight EKF Algorithm for State Estimation purposes ***************************************************/

#include "AttitudeEstimation.h"
#include "init.h"
#include "DefFuncs.h"
#include "math.h" 
#include "string.h"

void Attitude_init(VehicleAtd* atd)
{
	atd->qs = 1.0f;
	atd->qx = 0.0f;
	atd->qy = 0.0f;
	atd->qz = 0.0f;

	atd->qdelts = 0.0f;
	atd->qdeltx = 0.0f;
	atd->qdelty = 0.0f;
	atd->qdeltz = 0.0f;

	atd->timeStep = 15.0f/1000.0f;
}

void Attitude_newMeas(VehicleAtd* atd, sensors* meas)
{
	float measC[3];
	measC[0] = meas->meas[3] - meas->gyrCalx; 
	measC[1] = meas->meas[4] - meas->gyrCaly; 
	measC[2] = meas->meas[5] - meas->gyrCalz; 
	memcpy(&atd->gyr[0], &measC[0], 12);

//	sendS("f", measC[0]);sendS("c", "     ");sendS("f", measC[1]);sendS("c", "    ");sendS("f", measC[2]);sendS("c", "\n\r");
	atd->updateFlag = 1;
}

void Attitude_PredictQuat(VehicleAtd* atd)
{
	if (atd->updateFlag)
	{
		float dt = atd->timeStep;

		float w[3] = {atd->gyr[0], atd->gyr[1], atd->gyr[2]};
		float w_n = sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);

		w[0] = (w_n == 0.0f ? (w[0] + 0.01f) : w[0]);
		w[1] = (w_n == 0.0f ? (w[1] + 0.01f) : w[1]);
		w[2] = (w_n == 0.0f ? (w[2] + 0.01f) : w[2]);

		w_n = sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);

		float axis[3] = {w[0]/w_n, w[1]/w_n, w[2]/w_n};

		float angle = w_n*dt;

		atd->qdelts = cosf(angle/2.0f);
		atd->qdeltx = axis[0]*sinf(angle/2.0f);
		atd->qdelty = axis[1]*sinf(angle/2.0f);
		atd->qdeltz = axis[2]*sinf(angle/2.0f);

		atd->qtemps = atd->qs*atd->qdelts - atd->qx*atd->qdeltx - atd->qy*atd->qdelty - atd->qz*atd->qdeltz;

		atd->qtempx = atd->qs*atd->qdeltx + atd->qx*atd->qdelts + atd->qy*atd->qdeltz - atd->qz*atd->qdelty;

		atd->qtempy = atd->qs*atd->qdelty - atd->qx*atd->qdeltz + atd->qy*atd->qdelts + atd->qz*atd->qdeltx;

		atd->qtempz = atd->qs*atd->qdeltz + atd->qx*atd->qdelty - atd->qy*atd->qdeltx + atd->qz*atd->qdelts;

		float q_n = sqrt(atd->qtemps*atd->qtemps + atd->qtempx*atd->qtempx + atd->qtempy*atd->qtempy + atd->qtempz*atd->qtempz);

		atd->qs = atd->qtemps/q_n;
		atd->qx = atd->qtempx/q_n;
		atd->qy = atd->qtempy/q_n;
		atd->qz = atd->qtempz/q_n;


		atd->updateFlag = 0;
	}
}

void Attitude_QuatToEuler(VehicleAtd* atd)
{
	atd->euler[0] = 57.295f*atan2f(2.0f*(atd->qy*atd->qz + atd->qx*atd->qs), (atd->qs*atd->qs - atd->qx*atd->qx - atd->qy*atd->qy + atd->qz*atd->qz));
	atd->euler[1] = 57.295f*asinf(2.0f*(atd->qy*atd->qs - atd->qx*atd->qz));
	atd->euler[2] = 57.295f*atan2f(2.0f*(atd->qx*atd->qy + atd->qz*atd->qs), atd->qs*atd->qs + atd->qx*atd->qx - atd->qy*atd->qy - atd->qz*atd->qz);
}

void Attitude_run(VehicleAtd* atd)
{
	Attitude_PredictQuat(atd);
	Attitude_QuatToEuler(atd);


//	sendS("f", atd->gyr[0]);sendS("c", "      ");sendS("f", atd->gyr[1]);sendS("c", "      ");sendS("f", atd->gyr[2]);sendS("c", "      ");sendS("f", atd->qs);sendS("c", "      ");sendS("f", atd->qx);sendS("c", "      ");sendS("f", atd->qy);sendS("c", "      ");sendS("f", atd->qz);sendS("c", "\n\r"); 
//	sendS("f", atd->euler[0]);sendS("c", "      ");sendS("f", atd->euler[1]);sendS("c", "\n\r");
}
