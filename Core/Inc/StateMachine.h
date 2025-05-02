#ifndef INC_STATEMACHINE_H_
#define INC_STATEMACHINE_H_


typedef enum {
	BASE,
	GROUND,
	ARMED,
	FASTASCENT,
	SLOWASCENT,
	APOGEE,
	FREEFALL,
	LANDED
} StateTypes;

typedef void (*StateExec) (void);

typedef struct {
	StateTypes states;
	uint8_t currentState;
	StateExec StateHandlers[7];
	// 1. Max Altitude 2. Max velocity  3. Velocity of Landing 4. Time of Landing 5. Orientation of STEMnauts 6. STEMnaut survivabilty 7. Landing Temperature
	float	APRS[8];
} VehicleStates;

void BASE_Handle(void);
void GROUND_Handle(void);
void ARMED_Handle(void);
void FASTASCENT_Handle(void);
void SLOWASCENT_Handle(void);
void APOGEE_Handle(void);
void FREEFALL_Handle(void);
void LANDED_Handle(void);
void statemachine_init(void);
void statemachine_run(void);


#endif /* INC_STATEMACHINE_H_ */
