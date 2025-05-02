#ifndef INC_FEKF_H_
#define INC_FEKF_H_

#include "stdio.h"
#include "arm_math.h"
#include "Sensors.h"
#include "EKF_Helpers.h"
#include "AttitudeEstimation.h"



typedef struct {

        /* Holding several states determining matrices for 6 dimensional EKF  */
         arm_matrix_instance_f32 x, z, P, A, C, H, K, Q, R;

         /* Holding several intermediary calculation steps for the Kalman Filter */
         arm_matrix_instance_f32 At, PAt, APAt, Ht, PHt, HPHt, HPHtR, HPHtRI, HPHtRIHt, KH, IKH, IKHP, y, Ky, xtemp, Ptemp;

         /* Arming state of the rocket */
         uint8_t armed;

         /* Measurements */
         float meas[7];

	 /* Cleaned data */
	 float* f;

	 /* Updating data flag */
	 uint8_t updateFlag;

	/* Defining timestep */
	 float timeStep;

} fekf;

void FlightEKF_init(fekf* fkal, float Q, float R);
void FlightEKF_PredictStep(fekf* fkal, VehicleAtd* atd);
void FlightEKF_UpdateStep(fekf* fkal);
arm_status FlightEKF_KalmanGain(fekf* fkal);
arm_status FlightEKF_UpdateCovariancePred(fekf* fkal);
arm_status FlightEKF_UpdateCovarianceUpd(fekf* fkal);
arm_status FlightEKF_updateStates(fekf* fkal);
arm_status updateData(fekf* fkal);
uint8_t FlightEKF_Converged(fekf* fkal, sensors* meas);
void FlightEKF_CoriolisFree(fekf* fkal, VehicleAtd* atd);
void FlightEKF_TransitionFunction(fekf* fkal);
void FlightEKF_TransitionJacobian(fekf* fkal);
void FlightEKF_ObsMatrices(fekf* fkal);
void FlightEKF_newMeas(fekf* fkal, sensors* meas);
void FlightEKF_run(fekf* gkal, VehicleAtd* atd);

#endif /* INC_FEKF_H_ */


