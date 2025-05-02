#ifndef INC_BMP_280_H_
#define INC_BMP_280_H_

typedef struct{
	
	/* Trim Pairs fro calibration purposes */
	int16_t trimPar[12];

	/* Temperature Measurement */
	float bmpTemp;

	/* Pressure Measurement */
	float bmpPres;

	/* Calculated altitude */
	float bmpAlt;

	/* Altitude Calibration Value */
	float bmpAltCal;


} bmp280;

#define DEVICE_ADDRESS_BMP 0x76

#define WHO_AM_I_RETURN_BMP 96
#define ADDITIVE_OVRSMPL_PRES_1 0b00000100
#define ADDITIVE_OVRSMPL_PRES_2 0b00001000
#define ADDITIVE_OVRSMPL_PRES_4 0b00001100
#define ADDITIVE_OVRSMPL_PRES_8 0b00010000
#define ADDITIVE_OVRSMPL_PRES_16 0b00010100

#define ADDITIVE_OVRSMPL_TEMP_1 0b00100000
#define ADDITIVE_OVRSMPL_TEMP_2 0b01000000
#define ADDITIVE_OVRSMPL_TEMP_4 0b01100000
#define ADDITIVE_OVRSMPL_TEMP_8 0b10000000
#define ADDITIVE_OVRSMPL_TEMP_16 0b10100000

#define CONFIG 60

#define ADDITIVE_MODE_CONT 0b00000011

#define CTRLMES (ADDITIVE_OVRSMPL_TEMP_1 + ADDITIVE_OVRSMPL_PRES_4 + ADDITIVE_MODE_CONT)

#define sampleP 2000.0f

#define REG_WHO_AM_I_BMP 208
#define REG_CTRLMES_BMP 244
#define REG_CONFIG_BMP 245
#define REG_TRIMPAR_BMP 136
#define REG_DATA_PRES 247
#define REG_DATA_TEMP_BMP 250

#define KALMAN_PREDICT_TIME_ALT 10

void bmp280_init(bmp280* bmp);
void bmp280_PresCal(bmp280* bmp, uint16_t it);
void bmp280_PresTempRead(bmp280* bmp, uint8_t data[3]);
#endif /* INC_BMP_280_H_ */
