#ifndef INC_GEKF_H_
#define INC_GEKF_H_

#include "stdio.h"
#include "arm_math.h"
#include "Sensors.h"

typedef struct {
	
	/* Holding several state determining matrices for 7 dimensional EKF  */
	 arm_matrix_instance_f32 x, P, A, C, H, K, Q, R; 

	 /* Holding several intermediary calculation steps for the Kalman Filter */
	 arm_matrix_instance_f32 HP, HPHt, HPHtR, HPHtRI, HPHtRIHt, KH, IKH, IKHP, y, Ky, xtemp, Ptemp, Pdiff;

	 /* Arming state of the rocket */
	 uint8_t armed;

	 /* Measurements */
	 float meas[7];


} gekf;

void GroundEKF_init(gekf* kal, float Q, float R);
arm_status GroundEKF_PredictStep(gekf* gkal);
arm_status GroundEKF_KalmanGain(gekf* gkal);
arm_status GroundEKF_UpdateCovariance(gekf* gkal);
arm_status GroundEKF_UpdateStep(gekf* gkal);
uint8_t GroundEKF_Converged(gekf* gkal, sensors* meas);
void GroundEKF_newMeas(gekf* gkal, sensors* meas);
void GroundEKF_run(gekf* gkal, sensors* meas);

#endif /* INC_GEKF_H_ */

