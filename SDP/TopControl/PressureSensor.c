#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <fcntl.h>
#include "ms5803.h"
#include <stdint.h>

int main() {
	if (ms5803_init("/dev/i2c-1", 0x77) < 0 ){
		printf("MS5803 Initialization Failed.\n");
		return 0;
	}
	printf("MS5803 Initialization Successful\n");
	int32_t raw_temp;
	int32_t raw_pres;
	float temp_c;
	float pres_atm;
	while (1) {
		delay(100);
		ms5803_read(&raw_pres, &raw_temp);

		pres_atm = (float)raw_pres/(10000)*(0.986923);
		temp_c = (float)raw_temp/(100);
		//printf("pressure: %ld\ntemperature: %ld\n", pres/(10000),temp/(100));
		printf("pressure: %.2f\ntemperature: %.2f\n", pres_atm,temp_c);
	}
	ms5803_close();
	return 0;
	/*wiringPiSetup();
	int fd;
	fd = wiringPiI2CSetup(0x77);
	int r;
	r = write(fd, 0x1e, 1);
	if (r != -1){
		return -1;
	}
	//
	//wiringPiI2CWrite(fd,0x00);*/
}
