#include <stdio.h>
#include <stdlib.h>
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <errno.h>
#include <math.h>
#include <stdint.h>
#define PI	3.14159265359
#define dt104	0.009615384615
#define dt52	0.01923077
// LSM6DS3 CONFIGURATION
// --------------------------------------------------------------------------
#define LSM6DS3_DEVID 0x6b
#define LSM6DS3_DPS 2000
#define LSM6DS3_XLG 2

// LSM6DS3 REGISTER ADDRESSES
// -------------------------------------------------------------------------
#define LSM6DS3_G_CTRL2		0x11
#define LSM6DS3_G_OUTX		0x22
#define LSM6DS3_G_OUTY		0x24
#define LSM6DS3_G_OUTZ		0x26
#define LSM6DS3_XL_OUTX		0x28
#define LSM6DS3_XL_OUTY		0x2A
#define LSM6DS3_XL_OUTZ		0x2C
// 
#define LSM6DS3_G_FS_245	0b00000000
#define LSM6DS3_G_FS_500	0b00000100
#define LSM6DS3_G_FS_2000	0b00001100

#define LSM6DS3_G_ODR_416	0b01100000
#define LSM6DS3_G_ODR_104	0b01000000
#define LSM6DS3_G_ODR_52	0b00110000

int fd;
int DRDY_FLAG=0;
int TAP_FLAG=0;

int convertTwosCompToInt(int bin, int bits){
 	if (bin&(1<<(bits-1))){
	    bin = bin | ~((1<<bits)-1);
	}
	return bin;
}

void DRDY_ISR(void) {
	//if (wiringPiI2CReadReg8(fd,0x1C) & 0x07) {
	//	TAP_FLAG=1;
	//} 
	DRDY_FLAG=1;
	//printf("INTERRUPT\n");
}

int LSM6DS3_Setup(void) {
	int fd,num;
	float val;
	fd=wiringPiI2CSetup(LSM6DS3_DEVID);
	wiringPiI2CWriteReg8(fd,0x0D,0x00);
	wiringPiI2CWriteReg8(fd,LSM6DS3_G_CTRL2, 0x00);
	if (fd < 0) perror("I2C Setup Error");
	wiringPiI2CWriteReg8(fd,LSM6DS3_G_CTRL2, (LSM6DS3_G_ODR_52 | LSM6DS3_G_FS_2000));
	wiringPiI2CWriteReg8(fd,0x10,0x30);
	if(wiringPiISR(16,INT_EDGE_RISING,&DRDY_ISR)==-1) printf("ERROR\n");
	wiringPiI2CWriteReg8(fd,0x0D,0x02);
	return fd;
}
float LSM6DS3_AccReadX(fd){
	float val;
	int bitVal = wiringPiI2CReadReg16(fd,LSM6DS3_XL_OUTX);
	if (bitVal == -1) perror("Acc Read Error");
	else {
		bitVal = convertTwosCompToInt(bitVal,16);
		val = bitVal*LSM6DS3_XLG / ((float)(65536/2));
		return val;
	}
	return 0;
} 
float LSM6DS3_AccReadY(fd){
	float val;
	int bitVal = wiringPiI2CReadReg16(fd,LSM6DS3_XL_OUTY);
	if (bitVal == -1) perror("Acc Read Error");
	else {
		bitVal = convertTwosCompToInt(bitVal,16);
		val = bitVal*LSM6DS3_XLG / ((float)(65536/2));
		return val;
	}
	return 0;
} 
float LSM6DS3_AccReadZ(fd){
	float val;
	int bitVal = wiringPiI2CReadReg16(fd,LSM6DS3_XL_OUTZ);
	if (bitVal == -1) perror("Acc Read Error");
	else {
		bitVal = convertTwosCompToInt(bitVal,16);
		val = bitVal*LSM6DS3_XLG / ((float)(65536/2));
		return val;
	}
	return 0;
} 
float LSM6DS3_GyroReadX(fd){
	float val;
	int bitVal =(int16_t) wiringPiI2CReadReg16(fd,LSM6DS3_G_OUTX);
	printf("%d\n",bitVal);
	//if (bitVal == -1) perror("Gyro Read Error");
	//else {
	//	bitVal = convertTwosCompToInt(bitVal,16);
	//	val = (bitVal*LSM6DS3_DPS) / ((float)(65536/2));
	//	return val;	
	//}
	return 0;
}

float LSM6DS3_GyroReadY(fd){
	float val;
	int bitVal = wiringPiI2CReadReg16(fd,LSM6DS3_G_OUTY);
	if (bitVal==-1) perror("Gyro Read Error");
	else {
		bitVal = convertTwosCompToInt(bitVal,16);
		val = (bitVal*LSM6DS3_DPS) / ((float)(65536/2));
		return val;	
	}
	return 0;
}

float LSM6DS3_GyroReadZ(fd){
	float val;
	int bitVal = wiringPiI2CReadReg16(fd,LSM6DS3_G_OUTZ);
	if (bitVal == -1) perror("Gyro Read Error");
	else {
		bitVal = convertTwosCompToInt(bitVal,16);
		val = (bitVal*LSM6DS3_DPS) / ((float)(65536/2));
		return val;	
	}
	return 0;
}
// TEST HARNESS
// ------------------------------------------------------	
int main(){
	float valx,valy,valz;
	float xl_valx,xl_valy,xl_valz;
	float xAcc;
	float l_valx=0;
	float anglex=0;
	int i;
	float offset_anglex;
	if (wiringPiSetup() < 0) perror("WiringPi Lib Setup Error");
	fd = LSM6DS3_Setup();
	float offset_g_x = LSM6DS3_GyroReadX(fd);
	offset_anglex=atan2f((LSM6DS3_AccReadY(fd)),LSM6DS3_AccReadZ(fd))*180/PI;
	//anglex=-(offset_anglex);
//	wiringPiI2CWriteReg16(fd,0x2D,(wiringPiI2CReadReg16(fd,LSM6DS3_G_OUTX)));
//	wiringPiI2CWriteReg16(fd,0x2F,(wiringPiI2CReadReg16(fd,LSM6DS3_G_OUTY)));
	while(1){
		valx = LSM6DS3_GyroReadX(fd);
		//valy= LSM6DS3_GyroReadY(fd);
		//valz = LSM6DS3_GyroReadZ(fd);
//		if ((wiringPiI2CReadReg8(fd,0x3B)&0x10)==0){
//		valx = LSM6DS3_GyroReadX(fd);
//		valy= LSM6DS3_GyroReadY(fd);
//		valz = LSM6DS3_GyroReadZ(fd);
		//	printf("FIFO NOT EMPTY\n");
//		}
		if (DRDY_FLAG){				
	//		for (i=0;i<12;i++){
			xl_valx = LSM6DS3_AccReadX(fd);
			xl_valy = LSM6DS3_AccReadY(fd);	
			xl_valz = LSM6DS3_AccReadZ(fd);
			valx = LSM6DS3_GyroReadX(fd);//-offset_g_x;
			xAcc = (atan2f(xl_valy,xl_valz)*180/PI)-offset_anglex;
			//if((abs(valx-l_valx) != 0)||(abs(xAcc-anglex)>5)) {
				//if (xAcc < 0){
				//	xAcc = (180-abs(xAcc))+180;
				//}
			//	if (valx >= 498){
			//		anglex = (0.5*anglex+(valx*dt) + (0.5*xAcc));
			//	} else {
					anglex = (0.98*anglex+(valx*dt52) + (0.02*xAcc));
			//	}
				
				l_valx=valx;	
			//}
	//		//}
			printf("x-angle: %0.2f deg\n", anglex);
			printf("x-acc: %0.2f acc\n", xAcc);
			valy= LSM6DS3_GyroReadY(fd);
			valz = LSM6DS3_GyroReadZ(fd);
		//	printf("x-rotation: %0.3f dps\ny-rotation:%0.3f dps\nz-rotation: %0.3f dps\n", valx,valy, valz);
			DRDY_FLAG=0;
		}
		
	//	if (TAP_FLAG) {
	//		printf("Tap Event\n");
	//		TAP_FLAG=0;
	//	}
		//delay(100);
	}			
	return 0;
}
