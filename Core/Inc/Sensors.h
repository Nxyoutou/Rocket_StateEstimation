#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_


typedef struct {
	
	/* Measurements 1. accX 2. accY 3. accZ 4. gyrX 5. gyrY 6. gyrZ 7. altZ 8. temp */
	float meas[8];

	/* Sensor Biases */
	float accCalx;
	float accCaly;
	float accCalz;

	float gyrCalx;
	float gyrCaly;
	float gyrCalz;

	float AltCal;

	/* Registering first DMA Transfer */
	uint8_t FirstDMA;
	uint8_t flagBMP;
	uint8_t flagIMU;

} sensors;

void init_measurements(sensors* meas);
void make_measurements(sensors* meas);





#endif /* INC_SENSORS_H_ */

