/******************************************** This document contains the Ground EKF Algorithm for calibration purposes *************************************/

#include "GEKF.h"
#include "EKF_Helpers.h"
#include "arm_math.h"
#include "string.h"
#include "DefFuncs.h"

uint8_t converged;
float eps = 0.000001;


float PI7x7[7 * 7] = {1.0f, 0, 0, 0, 0, 0, 0,
                      0, 1.0f, 0, 0, 0, 0, 0,
                      0, 0, 1.0f, 0, 0, 0, 0,
                      0, 0, 0, 1.0f, 0, 0, 0,
                      0, 0, 0, 0, 1.0f, 0, 0,
                      0, 0, 0, 0, 0, 1.0f, 0,
                      0, 0, 0, 0, 0, 0, 1.0f};

float AI7x7[7 * 7] = {1.0f, 0, 0, 0, 0, 0, 0,
                      0, 1.0f, 0, 0, 0, 0, 0,
                      0, 0, 1.0f, 0, 0, 0, 0,
                      0, 0, 0, 1.0f, 0, 0, 0,
                      0, 0, 0, 0, 1.0f, 0, 0,
                      0, 0, 0, 0, 0, 1.0f, 0,
                      0, 0, 0, 0, 0, 0, 1.0f};

float HI7x7[7 * 7] = {1.0f, 0, 0, 0, 0, 0, 0,
                      0, 1.0f, 0, 0, 0, 0, 0,
                      0, 0, 1.0f, 0, 0, 0, 0,
                      0, 0, 0, 1.0f, 0, 0, 0,
                      0, 0, 0, 0, 1.0f, 0, 0,
                      0, 0, 0, 0, 0, 1.0f, 0,
                      0, 0, 0, 0, 0, 0, 1.0f};

float QI7x7[7 * 7] = {1.0f, 0, 0, 0, 0, 0, 0,
                      0, 1.0f, 0, 0, 0, 0, 0,
                      0, 0, 1.0f, 0, 0, 0, 0,
                      0, 0, 0, 1.0f, 0, 0, 0,
                      0, 0, 0, 0, 1.0f, 0, 0,
                      0, 0, 0, 0, 0, 1.0f, 0,
                      0, 0, 0, 0, 0, 0, 1.0f};

float RI7x7[7 * 7] = {1.0f, 0, 0, 0, 0, 0, 0,
                      0, 1.0f, 0, 0, 0, 0, 0,
                      0, 0, 1.0f, 0, 0, 0, 0,
                      0, 0, 0, 1.0f, 0, 0, 0,
                      0, 0, 0, 0, 1.0f, 0, 0,
                      0, 0, 0, 0, 0, 1.0f, 0,
                      0, 0, 0, 0, 0, 0, 1.0f};

float PI6x6[6 * 6] = {1.0f, 0, 0, 0, 0, 0,
                      0, 1.0f, 0, 0, 0, 0,
                      0, 0, 1.0f, 0, 0, 0,
                      0, 0, 0, 1.0f, 0, 0,
                      0, 0, 0, 0, 1.0f, 0,
                      0, 0, 0, 0, 0, 1.0f};

float AI6x6[6 * 6] = {1.0f, 0, 0, 0, 0, 0,
                      0, 1.0f, 0, 0, 0, 0,
                      0, 0, 1.0f, 0, 0, 0,
                      0, 0, 0, 1.0f, 0, 0,
                      0, 0, 0, 0, 1.0f, 0,
                      0, 0, 0, 0, 0, 1.0f};

float HI6x6[6 * 6] = {1.0f, 0, 0, 0, 0, 0,
                      0, 1.0f, 0, 0, 0, 0,
                      0, 0, 1.0f, 0, 0, 0,
                      0, 0, 0, 1.0f, 0, 0,
                      0, 0, 0, 0, 1.0f, 0,
                      0, 0, 0, 0, 0, 1.0f};

float QI6x6[6 * 6] = {1.0f, 0, 0, 0, 0, 0,
                      0, 1.0f, 0, 0, 0, 0,
                      0, 0, 1.0f, 0, 0, 0,
                      0, 0, 0, 1.0f, 0, 0,
                      0, 0, 0, 0, 1.0f, 0,
                      0, 0, 0, 0, 0, 1.0f};

float RI6x6[6 * 6] = {1.0f, 0, 0, 0, 0, 0,
                      0, 1.0f, 0, 0, 0, 0,
                      0, 0, 1.0f, 0, 0, 0,
                      0, 0, 0, 1.0f, 0, 0,
                      0, 0, 0, 0, 1.0f, 0,
                      0, 0, 0, 0, 0, 1.0f};


float K_init_7x7[7 * 7];
float K_init_6x6[6 * 6];

float x_init_7[7];
float C_init_7[7];
float x_init_6[6];
float C_init_6[6];

float z_init_6[6];

float HP7x7[7 * 7];
float HP6x6[6 * 6];

float HPHt7x7[7 * 7];
float HPHt6x6[6 * 6];

float HPHtR7x7[7 * 7];
float HPHtR6x6[6 * 6];

float HPHtRI7x7[7 * 7];
float HPHtRI6x6[6 * 6];

float HPHtRIHt7x7[7 * 7];
float HPHtRIHt6x6[6 * 6];

float KH7x7[7 * 7];
float KH6x6[6 * 6];

float IKH7x7[7 * 7];
float IKH6x6[6 * 6];

float IKHP7x7[7 * 7];
float IKHP6x6[6 * 6];

float y7[7];
float y6[6];

float Ky7[7];
float Ky6[6];

float xtemp7[7];
float xtemp6[6];

float Ptemp7x7[7 * 7];
float Ptemp6x6[6 * 6];


void GroundEKF_init(gekf* gkal, float Q, float R)
{
	arm_mat_init_f32(&gkal->x, 7, 1, x_init_7);

/*	gkal->x.pData[2] = -9.81f;  */
	/* Might need to include one barometer measurement to give a more accurate picture */
/*	gkal->x.pData[6] = 243.0f; */

	arm_mat_init_f32(&gkal->P, 7, 7, PI7x7);

	arm_mat_init_f32(&gkal->A, 7, 7, AI7x7);

	arm_mat_init_f32(&gkal->C, 7, 1, C_init_7);

	arm_mat_init_f32(&gkal->H, 7, 7, HI7x7);

	arm_mat_init_f32(&gkal->K, 7, 7, K_init_7x7);

	arm_mat_init_f32(&gkal->Q, 7, 7, QI7x7);

	arm_mat_init_f32(&gkal->R, 7, 7, RI7x7);
	for (int i = 0; i < 7; i++)
	{
		gkal->Q.pData[i*8] = gkal->Q.pData[i*8] * Q;
		gkal->R.pData[i*8] = gkal->R.pData[i*8] * R;
	}

	arm_mat_init_f32(&gkal->HPHtR, 7, 7, HPHtR7x7);
	arm_mat_init_f32(&gkal->HPHtRI, 7, 7, HPHtRI7x7);
	arm_mat_init_f32(&gkal->KH, 7, 7, KH7x7);
	arm_mat_init_f32(&gkal->IKH, 7, 7, IKH7x7);
	arm_mat_init_f32(&gkal->IKHP, 7, 7, IKHP7x7);
	arm_mat_init_f32(&gkal->Ptemp, 7, 7, Ptemp7x7);
	arm_mat_init_f32(&gkal->y, 7, 1, y7);
	arm_mat_init_f32(&gkal->Ky, 7, 1, Ky7);
	arm_mat_init_f32(&gkal->xtemp, 7, 1, xtemp7);

	gkal->armed = 0;

}

arm_status GroundEKF_PredictStep(gekf* gkal)
{
	arm_status status;
	status = ARM_MATH_SUCCESS;

	for (int i = 0; i < 7; i++) gkal->P.pData[i*8] += gkal->Q.pData[i*8]; 
//	sendMat(gkal->P.pData, 7, 7);newL();

	return status;

}

arm_status GroundEKF_KalmanGain(gekf* gkal)
{
	arm_status status;
	status = ARM_MATH_SUCCESS;


	/* R = H*P*H' + R */
	memcpy(gkal->HPHtR.pData, gkal->P.pData, (gkal->P.numCols*gkal->P.numRows*4));
	for (int i = 0; i < 7; i++) gkal->HPHtR.pData[i*8] += gkal->R.pData[i*8]; 
/*	sendMat(gkal->HPHtR.pData, 7, 7);sendS("c", "\n\r"); */

	/* R = (R)^-1 */
	status |= arm_mat_inverse_f32(&gkal->HPHtR, &gkal->HPHtRI); 
/*	sendMat(gkal->HPHtRI.pData, 7, 7);sendS("c", "\n\r"); */

	/* P*H'*R */
	status |= arm_mat_mult_f32(&gkal->P, &gkal->HPHtRI, &gkal->K); 
/*	sendMat(gkal->K.pData, 7, 7);sendS("c", "\n\r"); */
	


	return status;
}


arm_status GroundEKF_UpdateCovariance(gekf* gkal)
{
	arm_status status;
	status = ARM_MATH_SUCCESS;

	/* R = I - KH */
	memcpy(gkal->IKH.pData, gkal->K.pData, gkal->K.numCols*gkal->K.numRows*4);
	for (int i = 0; i < 7; i++) gkal->IKH.pData[i*8] = 1.0f - gkal->IKH.pData[i*8];
/*	sendMat(gkal->IKH.pData, 7, 7);sendS("c", "\n\r"); */

	/* P_n+1 = R*P_n */
	status |= arm_mat_mult_f32(&gkal->IKH, &gkal->P, &gkal->Ptemp);
	memcpy(gkal->P.pData, gkal->Ptemp.pData, (gkal->P.numCols * gkal->P.numRows*4));
/*	sendMat(gkal->P.pData, 7, 7);sendS("c", "\n\r"); */


	return status;

}

arm_status GroundEKF_UpdateStep(gekf* gkal)
{
	arm_status status;
	status = ARM_MATH_SUCCESS;

	/* y = Z - C */
	memcpy(gkal->C.pData, gkal->x.pData, (gkal->x.numCols * gkal->x.numRows * 4));
	for (int i = 0; i < 7; i++) gkal->y.pData[i] = gkal->meas[i] - gkal->C.pData[i];
/*	sendMat(&meas->meas[1], 1, 1);sendS("c", "     ");sendMat(&meas->meas[6], 1, 1);sendS("c", "\n\r"); */

	/* R = K*y */
	status |= arm_mat_mult_f32(&gkal->K, &gkal->y, &gkal->Ky);

	/* x_n+1 = x_n + R */
	for (int i = 0; i < 7; i++) gkal->xtemp.pData[i] = gkal->x.pData[i] + gkal->Ky.pData[i];

	memcpy(gkal->x.pData, gkal->xtemp.pData, (gkal->xtemp.numCols * gkal->xtemp.numRows * 4));

	return status;
}

uint8_t GroundEKF_Converged(gekf* gkal, sensors* meas)
{
	uint8_t converged_f = 1;	
//	sendMat(gkal->P.pData, 7, 7);newL();
	for (int i = 0; i < 7; i++)
	{
		if ((gkal->P.pData[i*8] - 0.095125f) > eps)//0.0952)0.105126)
		{
			converged_f = 0;
		}
	}

	if (converged_f) 
	{
		meas->accCalx = gkal->x.pData[0];
		meas->accCaly = gkal->x.pData[1];
		meas->accCalz = 9.81f + gkal->x.pData[2];
		meas->gyrCalx = gkal->x.pData[3];
		meas->gyrCaly = gkal->x.pData[4];
		meas->gyrCalz = gkal->x.pData[5];
		meas->AltCal = gkal->x.pData[6];
		gkal->armed = 1;
//		sendS("f", meas->accCalx);space();sendS("f", meas->accCaly);space();sendS("f", meas->accCalz);space();sendS("f", meas->gyrCalx);space();sendS("f", meas->gyrCaly);space();sendS("f", meas->gyrCalz);space();sendS("f", meas->AltCal);newL();






	}
	
	return converged_f;
}

void GroundEKF_newMeas(gekf* gkal, sensors* meas)
{
//	sendMat((float *) &meas->meas[0], 1, 7);newL();
	memcpy(&gkal->meas, &meas->meas, sizeof(gkal->meas));
}

void GroundEKF_run(gekf* gkal, sensors* meas)
{
	GroundEKF_PredictStep(gkal);
	GroundEKF_KalmanGain(gkal); 
	GroundEKF_UpdateCovariance(gkal);
	GroundEKF_UpdateStep(gkal);
	converged = GroundEKF_Converged(gkal, meas); 


//	sendS("f", gkal->x.pData[2]);sendS("c", "    ");sendS("f", gkal->x.pData[3]);sendS("c", "\n\r"); 
//	sendMat(&gkal->meas[0], 1, 7);sendS("c", "\n\r"); 
}
	
	

