#ifndef INC_STATES_H_ 
#define INC_STATES_H_

#include "stdio.h"
#include "Sensors.h"
#include "GEKF.h"
#include "AttitudeEstimation.h"
#include "FEKF.h"
#include "StateMachine.h"

#define BURN_TIME 6000

void BaseState(VehicleStates* State);
void GroundState(gekf* gkal, sensors* meas, VehicleStates* State);
void ArmedState(fekf* fkal, VehicleAtd* atd, VehicleStates* State);
void FastAscentState(fekf* fkal, VehicleAtd* atd, VehicleStates* State);
void SlowAscentState(fekf* fkal, VehicleAtd* atd, VehicleStates* State); 
void ApogeeState(fekf* fkal, VehicleAtd* atd, VehicleStates* State);
void FreefallState(fekf* fkal, VehicleAtd* atd, VehicleStates* State);
void LandedState(fekf* fkal, VehicleAtd* atd, VehicleStates* State);

#endif /* INC_STATES_H_ */
