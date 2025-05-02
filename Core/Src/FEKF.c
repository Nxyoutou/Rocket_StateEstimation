/************************** This document contains the Flight EKF Algorithm for State Estimation purposes ***************************************************/

#include "FEKF.h"
#include "arm_math.h"
#include "string.h"
#include "DefFuncs.h"
#include "init.h"
#include "StateMachine.h"

extern TIM_HandleTypeDef htim2;
extern VehicleStates States;

float dist[3] = {0.0f, 0.0f, 0.0f};

float cleanB[11]; 
float survive;

uint8_t firstIntegration = 1;
uint16_t last_upd;

float At[6 * 6];
float PAt[6 * 6];
float APAt[6 * 6];
float Ht[6 * 6];
float PHt[6 * 6];

void FlightEKF_init(fekf* fkal, float Q, float R)
{

	fkal->f = (float *) &cleanB;

 	arm_mat_init_f32(&fkal->x, 6, 1, x_init_6); 

	arm_mat_init_f32(&fkal->z, 6, 1, z_init_6);

        arm_mat_init_f32(&fkal->P, 6, 6, PI6x6);

        arm_mat_init_f32(&fkal->A, 6, 6, AI6x6);

        arm_mat_init_f32(&fkal->C, 6, 1, C_init_6);

        arm_mat_init_f32(&fkal->H, 6, 6, HI6x6);

        arm_mat_init_f32(&fkal->K, 6, 6, K_init_6x6);

        arm_mat_init_f32(&fkal->Q, 6, 6, QI6x6);

        arm_mat_init_f32(&fkal->R, 6, 6, RI6x6);

        arm_mat_init_f32(&fkal->At, 6, 6, At);

        arm_mat_init_f32(&fkal->PAt, 6, 6, PAt);

        arm_mat_init_f32(&fkal->APAt, 6, 6, APAt);

        arm_mat_init_f32(&fkal->Ht, 6, 6, Ht);

        arm_mat_init_f32(&fkal->PHt, 6, 6, PHt);

        arm_mat_init_f32(&fkal->HPHt, 6, 6, HPHt6x6);

        arm_mat_init_f32(&fkal->HPHtR, 6, 6, HPHtR6x6);

        arm_mat_init_f32(&fkal->HPHtRI, 6, 6, HPHtRI6x6);

	arm_mat_init_f32(&fkal->HPHtRIHt, 6, 6, HPHtRIHt6x6);

        arm_mat_init_f32(&fkal->HPHtRIHt, 6, 6, HPHtRIHt6x6);

        arm_mat_init_f32(&fkal->KH, 6, 6, KH6x6);

        arm_mat_init_f32(&fkal->IKH, 6, 6, IKH6x6);

        arm_mat_init_f32(&fkal->IKHP, 6, 6, IKHP6x6);

        arm_mat_init_f32(&fkal->y, 6, 1, y6);

        arm_mat_init_f32(&fkal->y, 6, 1, Ky6);

        arm_mat_init_f32(&fkal->y, 6, 1, xtemp6);

        arm_mat_init_f32(&fkal->y, 6, 1, Ptemp6x6);








        for (int i = 0; i < 7; i++)
        {
                fkal->Q.pData[i*8] = fkal->Q.pData[i*8] * Q;
                fkal->R.pData[i*8] = fkal->R.pData[i*8] * R;
        }

        arm_mat_init_f32(&fkal->HPHtR, 6, 6, HPHtR6x6);
        arm_mat_init_f32(&fkal->HPHtRI, 6, 6, HPHtRI6x6);
        arm_mat_init_f32(&fkal->KH, 6, 6, KH6x6);
        arm_mat_init_f32(&fkal->IKH, 6, 6, IKH6x6);
        arm_mat_init_f32(&fkal->IKHP, 6, 6, IKHP6x6);
        arm_mat_init_f32(&fkal->Ptemp, 6, 6, Ptemp6x6);
        arm_mat_init_f32(&fkal->y, 6, 1, y6);
        arm_mat_init_f32(&fkal->Ky, 6, 1, Ky6);
        arm_mat_init_f32(&fkal->xtemp, 6, 1, xtemp6);

        fkal->armed = 0; 

}

void FlightEKF_newMeas(fekf* fkal, sensors* meas)
{
	float measC[7];
	measC[0] = meas->meas[0] - meas->accCalx;
	measC[1] = meas->meas[1] - meas->accCaly;
	measC[2] = meas->meas[2] - meas->accCalz;
	measC[3] = meas->meas[3] - meas->gyrCalx;
	measC[4] = meas->meas[4] - meas->gyrCaly;
	measC[5] = meas->meas[5] - meas->gyrCalz;
	measC[6] = meas->meas[6] - meas->AltCal;

//	sendMat((float *) &meas->meas[0], 1, 7);newL();

	memcpy(&fkal->meas[0], &measC[0], sizeof(measC));
//	sendMat((float *) &measC[0], 1, 7);newL();
	fkal->updateFlag = 1;

}

void FlightEKF_CoriolisFree(fekf* fkal, VehicleAtd* atd)
{
	memcpy(fkal->f, &fkal->meas[0], 28);

	if (firstIntegration)
	{
		last_upd = __HAL_TIM_GET_COUNTER(&htim2);
		firstIntegration = 0;
	}


	uint16_t current_upd = __HAL_TIM_GET_COUNTER(&htim2);
	uint16_t dt = current_upd - last_upd;
	last_upd = current_upd;
	fkal->timeStep = (float) dt/1000000.0f;

	fkal->f[7]  = atd->qs;
	fkal->f[8]  = atd->qx;
	fkal->f[9]  = atd->qy;
	fkal->f[10] = atd->qz;
	float qs = fkal->f[7];
	float qx = fkal->f[8];
	float qy = fkal->f[9];
	float qz = fkal->f[10];
	fkal->z.pData[4] = fkal->meas[6];


	float q_mat[9] = {2.0f*(qs*qs + qx*qx) - 1.0f, 2.0f*(qx*qy + qz*qs)       , 2.0f*(qx*qz - qy*qs),
			  2.0f*(qx*qy - qz*qs)       , 2.0f*(qs*qs + qy*qy) - 1.0f, 2.0f*(qy*qz + qx*qs),
			  2.0f*(qx*qz + qy*qs)       , 2.0f*(qy*qz - qs*qx)       , 2.0f*(qs*qs + qz*qz) - 1.0f};

	float gx = 9.81f*q_mat[2];
	float gy = 9.81f*q_mat[5];
	float gz = 9.81f*q_mat[8];

	
	float wx = fkal->f[3];
	float wy = fkal->f[4];
	float wz = fkal->f[5];

	float coriolis_mat[9] = {((-1.0f)*wy*wy - wz*wz), (wy*wy)                , (wz*wz),
			         (wx*wy)                , ((-1.0f)*wx*wx - wz*wz), (wy*wz),
				 (wx*wz)                , (wy*wz)                , ((-1.0f)*wx*wx - wy*wy)};

	fkal->f[0] = fkal->f[0] + coriolis_mat[0]*dist[0] + coriolis_mat[1]*dist[1] + coriolis_mat[2]*dist[2] + gx;
	fkal->f[1] = fkal->f[1] + coriolis_mat[3]*dist[0] + coriolis_mat[4]*dist[1] + coriolis_mat[5]*dist[2] + gy;
	fkal->f[2] = fkal->f[2] + coriolis_mat[6]*dist[0] + coriolis_mat[7]*dist[1] + coriolis_mat[8]*dist[2] + gz; 

//	sendS("f", fkal->f[0]);space();sendS("f", fkal->f[1]);space();sendS("f", fkal->f[2]);newL();

}

void FlightEKF_TransitionFunction(fekf* fkal)
{

	float dt = fkal->timeStep;

	float qs = fkal->f[7];
	float qx = (-1.0f)*fkal->f[8];
	float qy = (-1.0f)*fkal->f[9];
	float qz = (-1.0f)*fkal->f[10];

	float q_mat[9] = {2.0f*(qs*qs + qx*qx) - 1.0f, 2.0f*(qx*qy + qz*qs)       , 2.0f*(qx*qz - qy*qs),
			  2.0f*(qx*qy - qz*qs)       , 2.0f*(qs*qs + qy*qy) - 1.0f, 2.0f*(qy*qz + qx*qs),
			  2.0f*(qx*qz + qy*qs)       , 2.0f*(qy*qz - qs*qx)       , 2.0f*(qs*qs + qz*qz) - 1.0f};

	float acc_inert_x = q_mat[0]*fkal->f[0] + q_mat[1]*fkal->f[1] + q_mat[2]*fkal->f[2];
	float acc_inert_y = q_mat[3]*fkal->f[0] + q_mat[4]*fkal->f[1] + q_mat[5]*fkal->f[2];
	float acc_inert_z = q_mat[6]*fkal->f[0] + q_mat[7]*fkal->f[1] + q_mat[8]*fkal->f[2];

	fkal->x.pData[1] = fkal->x.pData[1] + acc_inert_x*dt;
	fkal->x.pData[3] = fkal->x.pData[3] + acc_inert_y*dt;
	fkal->x.pData[5] = fkal->x.pData[5] + acc_inert_z*dt;

	fkal->x.pData[0] = fkal->x.pData[0] + fkal->x.pData[1]*dt;/* + 0.5f*acc_inert_x*dt*dt;*/
	fkal->x.pData[2] = fkal->x.pData[2] + fkal->x.pData[3]*dt;/* + 0.5f*acc_inert_x*dt*dt;*/
	fkal->x.pData[4] = fkal->x.pData[4] + fkal->x.pData[5]*dt;/* + 0.5f*acc_inert_x*dt*dt;*/
	
//	sendS("f", acc_inert_x);space();sendS("f", acc_inert_y);space();sendS("f", acc_inert_z);newL();
//	sendS("f", fkal->x.pData[4]);space();sendS("f", fkal->x.pData[5]);newL();

}

void FlightEKF_TransitionJacobian(fekf* fkal)
{
	
	float dt = fkal->timeStep;

	/* Jacobian of 1. and 2. states (x, vx) (1./2. Row) */
	fkal->A.pData[0] = 1.0f;
	fkal->A.pData[1] = dt;
	fkal->A.pData[2] = 0.0f;
	fkal->A.pData[3] = 0.0f;
	fkal->A.pData[4] = 0.0f;
	fkal->A.pData[5] = 0.0f;

	fkal->A.pData[6] = 0.0f;
	fkal->A.pData[7] = 1.0f;
	fkal->A.pData[8] = 0.0f;
	fkal->A.pData[9] = 0.0f;
	fkal->A.pData[10] = 0.0f;
	fkal->A.pData[11] = 0.0f;

	/* Jacobian of 3. and 4. states (y, yx) (3./4. Row) */
	fkal->A.pData[12] = 0.0f;
	fkal->A.pData[13] = 0.0f;
	fkal->A.pData[14] = 1.0f;
	fkal->A.pData[15] = dt;
	fkal->A.pData[16] = 0.0f;
	fkal->A.pData[17] = 0.0f;

	fkal->A.pData[18] = 0.0f;
	fkal->A.pData[19] = 0.0f;
	fkal->A.pData[20] = 0.0f;
	fkal->A.pData[21] = 1.0f;
	fkal->A.pData[22] = 0.0f;
	fkal->A.pData[23] = 0.0f;

	/* Jacobian of 5. and 6. states (y, yx) (5./6. Row) */
	fkal->A.pData[24] = 0.0f;
	fkal->A.pData[25] = 0.0f;
	fkal->A.pData[26] = 0.0f;
	fkal->A.pData[27] = 0.0f;
	fkal->A.pData[28] = 1.0f;
	fkal->A.pData[29] = dt;

	fkal->A.pData[30] = 0.0f;
	fkal->A.pData[31] = 0.0f;
	fkal->A.pData[32] = 0.0f;
	fkal->A.pData[33] = 0.0f;
	fkal->A.pData[34] = 0.0f;
	fkal->A.pData[35] = 1.0f;


}

arm_status FlightEKF_UpdateCovariancePred(fekf* fkal)
{
	arm_status status;
	status = ARM_MATH_SUCCESS;

	/* Calculate R = A*P*At */
	status |= arm_mat_trans_f32(&fkal->A, &fkal->At);
	status |= arm_mat_mult_f32(&fkal->P, &fkal->At, &fkal->PAt);
	status |= arm_mat_mult_f32(&fkal->A, &fkal->PAt, &fkal->APAt);
	memcpy(fkal->P.pData, fkal->APAt.pData, sizeof(float)*(fkal->P.numRows*fkal->P.numCols));
	
	/* Calculate R = R + Q */
	for (int i = 0; i < 6; i++) fkal->P.pData[i*7] += fkal->Q.pData[i*7];

	return status;
}	

void FlightEKF_ObsMatrices(fekf* fkal)
{
	fkal->C.pData[4] = fkal->x.pData[4];

	/* Jacobian of 1. and 2. states (x, vx) (1./2. Row) */
	fkal->H.pData[0] = 0.0f;
	fkal->H.pData[1] = 0.0f;
	fkal->H.pData[2] = 0.0f;
	fkal->H.pData[3] = 0.0f;
	fkal->H.pData[4] = 0.0f;
	fkal->H.pData[5] = 0.0f;

	fkal->H.pData[6] = 0.0f;
	fkal->H.pData[7] = 0.0f;
	fkal->H.pData[8] = 0.0f;
	fkal->H.pData[9] = 0.0f;
	fkal->H.pData[10] = 0.0f;
	fkal->H.pData[11] = 0.0f;

	/* Jacobian of 3. and 4. states (y, yx) (3./4. Row) */
	fkal->H.pData[12] = 0.0f;
	fkal->H.pData[13] = 0.0f;
	fkal->H.pData[14] = 0.0f;
	fkal->H.pData[15] = 0.0f;
	fkal->H.pData[16] = 0.0f;
	fkal->H.pData[17] = 0.0f;

	fkal->H.pData[18] = 0.0f;
	fkal->H.pData[19] = 0.0f;
	fkal->H.pData[20] = 0.0f;
	fkal->H.pData[21] = 0.0f;
	fkal->H.pData[22] = 0.0f;
	fkal->H.pData[23] = 0.0f;

	/* Jacobian of 5. and 6. states (y, yx) (5./6. Row) */
	fkal->H.pData[24] = 0.0f;
	fkal->H.pData[25] = 0.0f;
	fkal->H.pData[26] = 0.0f;
	fkal->H.pData[27] = 0.0f;
	fkal->H.pData[28] = 1.0f;
	fkal->H.pData[29] = fkal->timeStep;

	fkal->H.pData[30] = 0.0f;
	fkal->H.pData[31] = 0.0f;
	fkal->H.pData[32] = 0.0f;
	fkal->H.pData[33] = 0.0f;
	fkal->H.pData[34] = 0.0f;
	fkal->H.pData[35] = 0.0f;
}

arm_status FlightEKF_KalmanGain(fekf* fkal)
{
	arm_status status;
	status = ARM_MATH_SUCCESS;

	/* R = H*P*H' + R */
	status |= arm_mat_trans_f32(&fkal->H, &fkal->Ht);
	status |= arm_mat_mult_f32(&fkal->P, &fkal->Ht, &fkal->PHt);
	status |= arm_mat_mult_f32(&fkal->H, &fkal->PHt, &fkal->HPHt);

	memcpy(fkal->HPHtR.pData, fkal->HPHt.pData, sizeof(float)*(fkal->HPHt.numCols*fkal->HPHt.numRows));


        for (int i = 0; i < 6; i++) fkal->HPHtR.pData[i*7] += fkal->R.pData[i*7]; 
/*      sendMat(fkal->HPHtR.pData, 6, 6);sendS("c", "\n\r"); */

        /* R = (R)^-1 */
        status |= arm_mat_inverse_f32(&fkal->HPHtR, &fkal->HPHtRI); 
/*      sendMat(gkal->HPHtRI.pData, 7, 7);sendS("c", "\n\r"); */

        /* P*H'*R */
	status |= arm_mat_mult_f32(&fkal->Ht, &fkal->HPHtRI, &fkal->HPHtRIHt);
        status |= arm_mat_mult_f32(&fkal->P, &fkal->HPHtRIHt, &fkal->K); 
/*      sendMat(gkal->K.pData, 7, 7);sendS("c", "\n\r"); */




	return status;
}

arm_status FlightEKF_UpdateCovarianceUpd(fekf* fkal)
{
	arm_status status;
	status = ARM_MATH_SUCCESS;

	/* R = I - KH */
	status |= arm_mat_mult_f32(&fkal->K, &fkal->H, &fkal->KH);
	memcpy(fkal->IKH.pData, fkal->KH.pData, sizeof(float)*(fkal->IKH.numCols*fkal->IKH.numRows));
	for (int i = 0; i < 6; i++) fkal->IKH.pData[i*7] = 1.0f - fkal->IKH.pData[i*7];

	/* R = R*P */
	arm_mat_mult_f32(&fkal->IKH, &fkal->P, &fkal->Ptemp);
	memcpy(fkal->P.pData, fkal->Ptemp.pData, sizeof(float)*(fkal->P.numCols*fkal->P.numRows));


	return status;
}

arm_status FlightEKF_updateStates(fekf* fkal)
{
	arm_status status;
	status = ARM_MATH_SUCCESS;

	/* y = z - C */
	for(int i = 0; i < 7; i++) fkal->y.pData[i] = fkal->z.pData[i] - fkal->C.pData[i];

	/* x_n+1 = x_n + Ky */
	status |= arm_mat_mult_f32(&fkal->K, &fkal->y, &fkal->Ky);
	for (int i = 0; i < 7; i++) fkal->x.pData[i] += fkal->Ky.pData[i];


	return status;
}
		


void FlightEKF_PredictStep(fekf* fkal, VehicleAtd* atd)
{

	FlightEKF_CoriolisFree(fkal, atd);
	FlightEKF_TransitionFunction(fkal);

	
}

void FlightEKF_UpdateStep(fekf* fkal)
{
	FlightEKF_ObsMatrices(fkal);
	FlightEKF_KalmanGain(fkal);
	FlightEKF_UpdateCovarianceUpd(fkal);
	FlightEKF_updateStates(fkal);
}

void FlightEKF_run(fekf* fkal, VehicleAtd* atd)
{
	if (fkal->updateFlag)
	{
		FlightEKF_PredictStep(fkal, atd);
		FlightEKF_UpdateStep(fkal);
		survive = sqrt((fkal->f[0]*fkal->f[0]/1600) + (fkal->f[1]*fkal->f[1]/900) + (fkal->f[2]*fkal->f[2]/100));
//		sendMat(fkal->x.pData, 1, 6);newL();
//		sendS("f", fkal->x.pData[4]);space();sendS("f", fkal->x.pData[5]);newL();
//		sendMat(&fkal->meas[0], 1, 7);newL();
		if (fkal->x.pData[4] > States.APRS[0]) States.APRS[0] = fkal->x.pData[4];
		if (fkal->x.pData[5] > States.APRS[1]) States.APRS[1] = fkal->x.pData[5];
		if (survive > States.APRS[6]) States.APRS[6] = survive;
		fkal->updateFlag = 0;
	}
}
