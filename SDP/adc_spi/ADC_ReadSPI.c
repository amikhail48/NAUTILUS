
#include <wiringPi.h>
#include <wiringPiSPI.h>

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
int main(){
	int fd;
	uint8_t num=0;
	wiringPiSetup();
	fd=wiringPiSPISetup(0,90000);
	printf("%d\n",sizeof(num));
	uint8_t *data;
	data = malloc(sizeof(uint16_t));
       	*data=0b1101;
	//data[1]=0b10;
	//data[2]=1;
	printf("starting buffer:%d\n",*data);
	int result;
	while (1){
		result = wiringPiSPIDataRW(0,data,16);//read(fd,data,2);
		printf("read:%d\n%d\n",*data,result);
		delay(100);
		*data=0b1101;
	}
	return 0;

}
