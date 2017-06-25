#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>

#define BASEDC	20
#define T_PITCH	0
#define T_ROLL	0

// COEFFICIENTS
#define KP	1
#define KI	1
#define KD	1

int main() {
	LSM6DS3_Init();
	//init motors to baseline duty cycle 
	int currAxis[3];
	int y_err_p;
	int y_err_i=0;
	int y_err_d;
	int y_err_prev=0;
	int x_out;
	int y_out;
	while(1){
		currAxis = LSM6DS3_GetAxis();
		y_err_p = currAxis[1]-T_ROLL;
		y_err_i += y_err_p;
		y_err_d = y_err_prev - y_err_p;
		y_err_prev = y_err_p;
		y_out =	KP*(y_err_p) + KI*(y_err_i) + KD*(y_err_d);
		delay(900);		
	}

	return 0;
} 
