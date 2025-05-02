#ifndef INC_EKF_HELPERS_H_
#define INC_EKF_HELPERS_H_

extern float PI7x7[7 * 7];
	     	      
extern float AI7x7[7 * 7];
	     	      
extern float HI7x7[7 * 7];
	     	      
extern float QI7x7[7 * 7];
	     	      
extern float RI7x7[7 * 7];

	     	      
extern float PI6x6[6 * 6];

extern float AI6x6[6 * 6];
	     	      
extern float HI6x6[6 * 6];
	     	      
extern float QI6x6[6 * 6];
	     	      
extern float RI6x6[6 * 6];
	     	      

extern float K_init_7x7[7 * 7];
extern float K_init_6x6[6 * 6];

extern float x_init_7[7];
extern float C_init_7[7];
extern float x_init_6[6];
extern float C_init_6[6];

extern float z_init_6[6];

extern float HP7x7[7 * 7];
extern float HP6x6[6 * 6];

extern float HPHt7x7[7 * 7];
extern float HPHt6x6[6 * 6];

extern float HPHtR7x7[7 * 7];
extern float HPHtR6x6[6 * 6];

extern float HPHtRI7x7[7 * 7];
extern float HPHtRI6x6[6 * 6];

extern float HPHtRIHt7x7[7 * 7];
extern float HPHtRIHt6x6[6 * 6];

extern float KH7x7[7 * 7];
extern float KH6x6[6 * 6];

extern float IKH7x7[7 * 7];
extern float IKH6x6[6 * 6];

extern float IKHP7x7[7 * 7];
extern float IKHP6x6[6 * 6];

extern float y7[7];
extern float y6[6];

extern float Ky7[7];
extern float Ky6[6];

extern float xtemp7[7];
extern float xtemp6[6];

extern float Ptemp7x7[7 * 7];
extern float Ptemp6x6[6 * 6];






#endif /* INC_EKF_HELPERS_H_ */
