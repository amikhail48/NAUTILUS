#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <errno.h>
#include <math.h>
#include </usr/include/linux/i2c-dev.h>
#include <pigpio.h>
#define PI	3.14159265359
#define dt104	0.009615384615
#define dt52	0.01923077
#define dt416	0.0024038461538462
#define dt833	0.00120048019

#define dt	dt416
// LSM6DS3 CONFIGURATION
// --------------------------------------------------------------------------
#define LSM6DS3_DEVID 0x6b
#define LSM6DS3_DPS 500
#define LSM6DS3_XLG 2

#define NUM_SAMPLES 90 

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

int FIFO_FLAG=0;
void DRDY_ISR(void){

	FIFO_FLAG=1;
}

int fd;
int DRDY_FLAG=0;
int TAP_FLAG=0;

int convertTwosCompToInt(int bin, int bits){
 	if (bin&(1<<(bits-1))){
	    bin = bin | ~((1<<bits)-1);
	}
	return bin;
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
	int bitVal = wiringPiI2CReadReg16(fd,LSM6DS3_G_OUTX);
	if (bitVal == -1) perror("Gyro Read Error");
	else {
		bitVal = convertTwosCompToInt(bitVal,16);
		val = (bitVal*LSM6DS3_DPS) / ((float)(65536/2));
		return val;	
	}
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
	if (bitVal==-1) perror("Gyro Read Error");
	else {
		bitVal = convertTwosCompToInt(bitVal,16);
		val = (bitVal*LSM6DS3_DPS) / ((float)(65536/2));
		return val;	
	}
	return 0;
}
float LSM6DS3_ReadFIFO(fd){
	float val;
	int bitVal = wiringPiI2CReadReg16(fd,0x3e);
	if (bitVal==-1) perror("FIFO Read Error");
	else {
		bitVal = convertTwosCompToInt(bitVal,16);
		if (bitVal < 0) {
			val = bitVal/(float)(32768);
		} else {
			val = bitVal/(float)(32767);
		}
		//val = (bitVal)/ ((float)(65535/2));
		return val;
	} return 0;
}

int combineCharsToInt(char a, char b){
	int c;
	c = (a<<8)+b;
	return c;
}
int main(){
    int num;
    float xl_valx,xl_valy,xl_valz;
    float gy_valx,gy_valy,gy_valz;
    float angle_x=0,angle_y=0,angle_z=0;
    float avg_xl_valx, avg_xl_valy, avg_xl_valz;
    float xAcc;
    int negative;
    int fd;
    int i;
    char * data;
    wiringPiSetup();
    gpioInitialise();
    if ((fd=wiringPiI2CSetup(LSM6DS3_DEVID))==-1) printf("Failed");
    //if (num==-1){
//	perror("Error:");
  //  } else 
//	printf("%d\n",num);
    if(wiringPiISR(16,INT_EDGE_RISING,&DRDY_ISR)==-1) printf("ERROR\n");
    //wiringPiI2CWriteReg8(fd,0x11,0x00);
    //wiringPiI2CWriteReg8(fd,0x10,0x00);
    wiringPiI2CWriteReg8(fd,0x12,0x44);
    wiringPiI2CWriteReg8(fd,0x0d,0x00);
    wiringPiI2CWriteReg8(fd,0x0e,0x08); // fifo threshold
    wiringPiI2CWriteReg8(fd,0x0a,0x26);
    int fifo_num = wiringPiI2CReadReg16(fd,0x3A);
    //fifo_num = (fifo_num&(0b0000111111111111));
    printf ("Emptying FIFO of %d bytes...",fifo_num);
    while ((wiringPiI2CReadReg8(fd,0x3B)&(0x10)) == 0){
	wiringPiI2CReadReg16(fd,0x3e);
    }
    printf(" DONE\n");
    wiringPiI2CWriteReg8(fd,0x0a,0x38);
    wiringPiI2CWriteReg8(fd,0x11,0b01100100);
    wiringPiI2CWriteReg8(fd,0x10,0b01010000);
    wiringPiI2CWriteReg8(fd,0x08,0x0A);
    wiringPiI2CWriteReg8(fd,0x06,0x20);
    wiringPiI2CWriteReg8(fd,0x0a,0b00110110);// ODR=416, CONTINUOUS MODE
    
    float lgy_valx;
    float offset_gy_x=0,offset_gy_y=0,offset_gy_z=0;
    float offset_xl_x=0;
    float offset_xl_y=0;
    float offset_xl_z=0;
    float fromFIFO;
    int startpoint; 
    printf("offset: %0.2f\n",offset_gy_x);
    // ---------------------------------------------------------
    // START CALIBRATION
    // ---------------------------------------------------------
    while (FIFO_FLAG == 0){
	 ; 
    }
    printf("Beginning Calibration\nDo not touch device.\n");
    startpoint = wiringPiI2CReadReg8(fd,0x3c);
    for (i=0;i<9;i++){
	//printf("%d\n",startpoint);
		
	fromFIFO = LSM6DS3_ReadFIFO(fd);
	//printf("%0.2f\n",fromFIFO);
	switch(startpoint){
		case 0:
		case 6:
		case 9:
		case 12:
//		case 15:
//		case 18:
//		case 21:
		
		//      printf("accumulate to x gyro\n");
			//if ((fromFIFO>0.00078)||(fromFIFO<=0) ){
			offset_gy_x += ((LSM6DS3_DPS*fromFIFO))*dt;
			//}
			break;
		case 1:
		case 7:
		case 10:
		case 13:
//		case 16:
//		case 19:
//		case 22:
		
		//	printf("accumulate to y gyro\n");//accumulate to y gyro
			offset_gy_y += LSM6DS3_DPS*fromFIFO*dt;
			break;
		case 2:
		case 8:
		case 11:
		case 14:
//		case 17:
//		case 20:
//		case 23:
	
		//	printf("accumulate to z gyro\n");
			offset_gy_z += LSM6DS3_DPS*fromFIFO*dt;
			break;
		case 3:
		//	printf("accumulate to  x acc\n");
			offset_xl_x += (LSM6DS3_XLG*fromFIFO);
			break;
		case 4:
		//	printf("accumulate to y acc\n");
			offset_xl_y += LSM6DS3_XLG*fromFIFO;
			break;
		case 5:
		//	printf("accumulate to z acc\n");
			offset_xl_z += LSM6DS3_XLG*fromFIFO;
			break;
		default:
			printf("Error fifo misalignment\n");
			break;
        }
	if (startpoint<8){
		startpoint++;
	} else {
		startpoint=0;
	}
	}
    FIFO_FLAG=0;
    offset_xl_x = offset_xl_x;
    offset_xl_y = offset_xl_y;
    offset_xl_z = (float)1-offset_xl_z;
    printf("Gyro X Offset: %0.4f\n", offset_gy_x);
    printf("Gyro Y Offset: %0.4f\n", offset_gy_y);
    printf("Acc Y Offset: %0.4f\n", offset_xl_y);
    printf("Acc Z Offset: %0.4f\n", offset_xl_z);
    
    int testVal=0;
    int pfd = i2cOpen(1,0x6b,0);
    char *blockFIFO;
    blockFIFO=(char*)malloc(sizeof(char)*32);
    int byteSum=0;    
    // -----------------------------
    // START MAIN LOOP
    // -----------------------------
    while(1) {
	fifo_num = wiringPiI2CReadReg16(fd,0x3a);
	//printf("%d bytes in FIFO\n",fifo_num);
		
	if (FIFO_FLAG){
	xl_valx=0;xl_valy=0;xl_valz=0;
	gy_valx=0;gy_valy=0;gy_valz=0;
	startpoint = wiringPiI2CReadReg8(fd,0x3c);
	printf("start: %d\n",startpoint);
	while (byteSum < 256){
	num=i2cReadI2CBlockData(pfd,0x3e,blockFIFO,18);
	printf("Read:%d bytes\n",num);
	fifo_num = wiringPiI2CReadReg16(fd,0x3a);
	printf("%d bytes in FIFO\n",fifo_num);
	if (fifo_num >= 18) {
		byteSum += num;
	} else {
		byteSum = 257;
	}
	// GYRO VALUES
 	for (i=0; i<18; i++){
	testVal = combineCharsToInt(blockFIFO[i+1], blockFIFO[i]);
	testVal = convertTwosCompToInt(testVal,16);
	//gy_valx += dt*((float)LSM6DS3_DPS*(float)convertTwosCompToInt(testVal,16)/(float)(32767));//-offset_gy_x);	
		switch(startpoint){
			case 0:
			case 6:
				gy_valx += testVal*dt*LSM6DS3_DPS/(float)32767;
				break;
			case 1:
			case 7:
				gy_valy += dt*LSM6DS3_DPS/(float)32767;
				break;
			case 2:
			case 8:
				gy_valz += dt*LSM6DS3_DPS/(float)32767;
				break;
			case 3:
				xl_valx = LSM6DS3_XLG
	//	for (i=0;i<1;i++){
	//	testVal = combineCharsToInt(blockFIFO[1], blockFIFO[0]);
	//	printf("%d\n",testVal);
	//	gy_valx = gy_valx+ dt*((float)LSM6DS3_DPS*(float)convertTwosCompToInt(testVal,16)/(float)(32767));//- offset_gy_x);
	//	printf("%d: %0.2f\n", i, gy_valx);
	}
        gy_valx = gy_valx-offset_gy_x;
        //fromFIFO = (((*blockFIFO)&(0xff))<<8)|( *(blockFIFO+sizeof(char)) );
        //fromFIFO = convertTwosCompToInt(fromFIFO,16);
	//printf("block x-gy: %d\n",fromFIFO);
        }
	byteSum=0;
        printf("DONE\n");
        gy_valy = gy_valy-offset_gy_y;
        avg_xl_valx = xl_valx;///(NUM_SAMPLES/15);
	avg_xl_valy = xl_valy;///(float)(NUM_SAMPLES/15);
	avg_xl_valz = xl_valz;///(NUM_SAMPLES/15);
        printf("x-gy: %0.4f\n", gy_valx);
	//printf("x-acc: %0.4f\n", avg_xl_valx);
        //printf("y-acc: %0.4f\n", avg_xl_valy);
	//printf("z-acc: %0.4f\n", avg_xl_valz);
        angle_x = angle_x + gy_valx;
	//angle_x=0.97*(angle_x+gy_valx)+(0.03)*(atan2f(avg_xl_valy,avg_xl_valz) * 180 / PI);
	angle_y=0.97*(angle_y-gy_valy)+(0.03)*(atan2f(avg_xl_valx,(avg_xl_valz)) * 180 / PI);	
	printf("x-axis: %0.4f\n",angle_x);
	printf("y-axis: %0.4f\n",angle_y);
	
        FIFO_FLAG=0;
        }
	} 
    return 0;
}

