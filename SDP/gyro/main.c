#include <wiringPiI2C.h>
#include <stdlib.h>
#include <stdio.h>
#include <wiringPi.h>
#include <pigpio.h>
#include <errno.h>
#define XL_DEVID 0x1d
int main(){
    int num;
    float x_g;
    int negative;
    int fd;
    char * data;
    if ((fd=wiringPiI2CSetup(XL_DEVID))==-1) printf("Failed");
    num = wiringPiI2CReadReg16(fd,40);
    if (num==-1){
	perror("Error:");
    } else 
	printf("%d\n",num);
    while(1) {
        num = wiringPiI2CReadReg16(fd,40);
	negative = (num&(1<<15))!=0;
 	if (negative){
	    num = num | ~((1<<16)-1);
	}
        x_g = (num*2.0) / (float)(65536/2);
        printf("x-axis: %0.4f\n",x_g);
        //num = wiringPiI2CReadReg16(fd,42);
	//printf("y-axis: %d\n",num);
        delay(100);
    } 
     //   num = wiringPiSPIDataRW(0,data,0);
//	printf("%d\n",data);
//	//time_sleep(1);
  //  }
    return 0;
}


