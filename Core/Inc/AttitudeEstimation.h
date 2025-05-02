#ifndef INC_ATTITUDEESTIMATION_H_
#define INC_ATTITUDEESTIMATION_H_

#include "stdio.h"
#include "Sensors.h"

typedef struct {

        /* Defining delta quaternion states  */
        float qdelts, qdeltx, qdelty, qdeltz; 

         /* Holding intermediary quaternion values*/
	float qtemps, qtempx, qtempy, qtempz;

	/* Holding current Quaternions */
	float qs, qx, qy, qz;

         /* Holding Gyroscope measurements */
         float gyr[3];

	 /* Updating data flag */
	 uint8_t updateFlag;

	 /* Defining Euler Angles */
	 float euler[3];

	 /* Defining timestep */
	 float timeStep;



} VehicleAtd;

void Attitude_init(VehicleAtd* atd);
void Attitude_newMeas(VehicleAtd* atd, sensors* meas);
void Attitude_PredictQuat(VehicleAtd* atd);
void Attitude_QuatToEuler(VehicleAtd* atd);
void Attitude_run(VehicleAtd* atd);

#endif /* INC_ATTITUDEESTIMATION_H_ */

