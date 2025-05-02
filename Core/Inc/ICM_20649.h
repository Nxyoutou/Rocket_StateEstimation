#ifndef INC_ICM_20649_H_
#define INC_ICM_20649_H_

#include "stdio.h"

typedef struct {

	/* Clean Gyroscope data in (rad/s) */
	float GyroM[3];

	/* Clean Accelerometer data in (m/s^2) */
	float AccM[3];

	/* Declaring Calibrated values */
	float GyroCal[3];
	float AccCal[3];

	uint8_t allowTrigger;

} icm;

/*********************************************************************************/
// In this system, registers are divided into different USER_BANKS
// The Device Identification, Power Management, and Sensor Readings are in USER_BANK_0
// The GYR and ACC Configurations are in USER_BANK_2
// The Register to change the USER_BANK is always 127
/*********************************************************************************/

#define DEVICE_ADDRESS 0x68

#define USR_BANK_0 0
#define USR_BANK_1 16
#define USR_BANK_2 32
#define USR_BANK_3 48

#define ADDITIVE_LPF_0 0b00000000
#define ADDITIVE_LPF_1 0b00001000
#define ADDITIVE_LPF_2 0b00010000
#define ADDITIVE_LPF_3 0b00011000
#define ADDITIVE_LPF_4 0b00100000
#define ADDITIVE_LPF_5 0b00101000
#define ADDITIVE_LPF_6 0b00110000
#define ADDITIVE_LPF_7 0b00111000

#define ADDITIVE_FS_GYR_500  0b00000000
#define ADDITIVE_FS_GYR_1000 0b00000010
#define ADDITIVE_FS_GYR_2000 0b00000100
#define ADDITIVE_FS_GYR_4000 0b00000110

#define ADDITIVE_FS_ACC_4G  0b00000000
#define ADDITIVE_FS_ACC_8G 0b00000010
#define ADDITIVE_FS_ACC_16G 0b00000100
#define ADDITIVE_FS_ACC_30G 0b00000110


#define ADDITIVE_LPF_ACTIVE 0b00000001
#define ADDITIVE_LPF_INACTIVE 0b00000000


#define ADDITIVE_OVRSMPL_TEMP_1 0b00100000
#define ADDITIVE_OVRSMPL_TEMP_2 0b01000000
#define ADDITIVE_OVRSMPL_TEMP_4 0b01100000
#define ADDITIVE_OVRSMPL_TEMP_8 0b10000000
#define ADDITIVE_OVRSMPL_TEMP_16 0b10100000

/*********VALUES***********/

#define WHO_AM_I_RETURN_ICM 225

#define PWR_MGMT_INIT 193 // USER_BANK_0
#define PWR_MGMT_NORM 1
#define USR_CTRL_DEF 16

#define GYR_SMPLRT_DIV 8 // USER_BANK_2
#define GYR_CONFIG_I (ADDITIVE_LPF_3 | ADDITIVE_FS_GYR_4000 | ADDITIVE_LPF_ACTIVE)
#define GYR_CONFIG_II 0
#define ODR_ALIGN_EN 1
#define ACC_SMPLRT_DIV_I 0
#define ACC_SMPLRT_DIV_II 0
#define ACC_CONFIG_I (ADDITIVE_LPF_3 | ADDITIVE_FS_ACC_30G | ADDITIVE_LPF_ACTIVE)
#define ACC_CONFIG_II 0


/******REGISTERS***********/

#define REG_USR_BANK 127 // Universal
			 
#define REG_WHO_AM_I_ICM 0 // USER_BANK_0
#define REG_USER_CTRL 3 
#define REG_PWR_MGMT 6 
#define REG_ACC_DATA 45
		         
		         
#define REG_GYR_SMPLRT 0 // USER_BANK_2
#define REG_GYR_CONFIG_I 1 
#define REG_GYR_CONFIG_II 2
#define REG_ODR_ALIGN 9
#define REG_ACC_SMPLRT_I 16
#define REG_ACC_SMPLRT_II 17
#define REG_ACC_CONFIG_I 20
#define REG_ACC_CONFIG_II 21

void user_bank(uint8_t USR_BANK);
void icm20649_init(icm* kal);
void icm20649_DataRead(icm* kal, uint8_t* data);

#endif /* INC_ICM_20649_H_ */
