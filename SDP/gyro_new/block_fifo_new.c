#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include "LinkedListStruct.h"
#include <signal.h>
#include </usr/include/linux/i2c-dev.h>
#include <pigpio.h>
#include <unistd.h>
//#include <pthread.h>

#define PI	3.14159265359
#define dt104	0.009615384615
#define dt52	0.01923077
#define dt416	0.00240384615
#define dt833	0.00120048019

#define dt	dt416

// LSM6DS3 CONFIGURATION
// --------------------------------------------------------------------------
#define LSM6DS3_DEVID 0x6b
#define LSM6DS3_DPS 500
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
int BLOCK_READY=0;
int FIFO_FLAG=0;
int fd;
char CAL_MODE=1;
int pfd;
int DRDY_FLAG=0;
int TAP_FLAG=0;
int IN_ISR=0;
char startpoint=0;
uint8_t FIFO_INT_STATUS;
LinkedList A;
char *blockFIFO;
char *targetBlock;
float currAxis[3];
float angle_x=0,angle_y=0,angle_z=0;
//void PIGIO_ISR(int gpio, int level, uint32_t tick){
void DRDY_ISR(void){
	//if (BLOCK_READY < 1) {
	//printf("enter isr\n");
	++BLOCK_READY;
	printf("exit isr\n");
	//}
}


int convertTwosCompToInt(int bin, int bits){
 	if (bin&(1<<(bits-1))){
	    bin = bin | ~((1<<bits)-1);
	}
	return bin;
}
int combineCharsToInt(char a, char b){
	int c;
	c = (a<<8)+b;
	return c;
}

void LSM6DS3_EmptyFIFO(int fd){
	
    	uint16_t fifo_num = (i2cReadWordData(pfd,0x3A)&(0x0FFF));
	printf("Emptying FIFO of %d bytes ",fifo_num);
	while ((i2cReadByteData(fd,0x3B)&(0x10)) == 0) {
		printf(".");
		i2cReadWordData(fd,0x3e);
	}
	printf("Done\n");
}    
void LSM6DS3_Setup(int fd){
    wiringPiI2CWriteReg8(fd,0x12,0x44);
    wiringPiI2CWriteReg8(fd,0x0d,0x00);
    wiringPiI2CWriteReg8(fd,0x0e,0x08); 
    wiringPiI2CWriteReg8(fd,0x0a,0x26);
    
    LSM6DS3_EmptyFIFO(pfd);
    wiringPiI2CWriteReg8(fd,0x0a,0x30);
    wiringPiI2CWriteReg8(fd,0x11,0b01100100);
    wiringPiI2CWriteReg8(fd,0x10,0b01000000);
    wiringPiI2CWriteReg8(fd,0x08,0b00001100); //gyro no decimation, xl decimation 4
    wiringPiI2CWriteReg8(fd,0x07,0x00);
    wiringPiI2CWriteReg8(fd,0x06,0x90); //set FTH
    wiringPiI2CWriteReg8(fd,0x0a,0b00110110);// ODR=416, CONTINUOUS MODE
}

void LSM6DS3_GetAxis(float* buf) {
	*(buf) = angle_x;
	*(buf+sizeof(int)) = angle_y;
	*(buf+sizeof(int)*2) = angle_z;
}
void stop(int signum) {
    printf("Freeing buffer...\n");
    freeLinkedList(&A);
    free(targetBlock); 
    free(blockFIFO);
}
// TEST HARNESS
int main(){
    int num;
    A = newLinkedList();
    float xl_valx,xl_valy,xl_valz;
    float gy_valx,gy_valy,gy_valz;
    float avg_xl_valx, avg_xl_valy, avg_xl_valz;
    float xAcc;
    int negative;
    int fd;
    int i;
    char data[32];
    blockFIFO = malloc(sizeof(char)*96);
    //char * targetBlock;
    targetBlock=malloc(sizeof(char)*30);
    //struct NodeObj targetNode;
    wiringPiSetup();
    gpioInitialise();
    if (piHiPri(99u) < 0) {perror("Priority Error: ");}
    gpioSetSignalFunc(SIGINT, stop);
    int pfd = i2cOpen(1,0x6b,0);
    if ((fd=wiringPiI2CSetup(LSM6DS3_DEVID))<0) printf("Failed");
    
    //gpioSetISRFunc(15,RISING_EDGE,-1,PIGIO_ISR);
    if(wiringPiISR(16,INT_EDGE_RISING,&DRDY_ISR)<0) printf("ERROR\n");
    LSM6DS3_Setup(fd);
    //wiringPiI2CWriteReg8(fd,0x11,0x00);
    //wiringPiI2CWriteReg8(fd,0x10,0x00);
    float lgy_valx;
    float offset_gy_x=0,offset_gy_y=0,offset_gy_z=0;
    float offset_xl_x=0;
    float offset_xl_y=0;
    float offset_xl_z=0;
    int curVal;
    int testVal=0;
    startpoint = -1;
    FIFO_FLAG=0;
    emptyList(A);  
    
    //wiringPiI2CWriteReg8(fd,0x0a,0b00110110);// ODR=416, CONTINUOUS MODE
    
    int prev_angle_x;    
    int j;
    i=0;  		
    int k=0;
    uint16_t fnum;
    static char thisStart;
	
    // -----------------------------
    // START MAIN LOOP
    // -----------------------------
    while(1) {
	//printf("%d bytes in FIFO\n",fifo_num);
	if (BLOCK_READY>0) {
		BLOCK_READY--;
		k=0;
		fnum = (uint16_t)((i2cReadWordData(pfd,0x3A))&(0x0FFF)) ;
		printf("start - fnum: %d\n", fnum);
		if (startpoint == -1)
			thisStart = i2cReadByteData(pfd,0x3c);
		while(fnum >= 30){
			++k;
		//for (k=0;k<3;k++){
			printf("loop %d: ",k);
			i2cReadI2CBlockData(pfd, 0x3E, (blockFIFO),30u); 
			printf("read - fnum: %d\n",fnum);
			addNode(A, blockFIFO, thisStart);
			/*if (thisStart < 14) {
				thisStart++;
			} else {
				thisStart=0;
			} */ 
			fnum = (uint16_t)((i2cReadWordData(pfd,0x3A))&(0x0FFF)) ;
			//fnum = (uint16_t)(((i2cReadByteData(pfd,0x3B)&(0x0F))<<8)+i2cReadByteData(pfd,0x3A));
			//printf("fnum: %d\n",fnum);
			++FIFO_FLAG;
		}
		printf("Done - fnum: %d\n",fnum);
	}
	
	if (FIFO_FLAG>0){
	printf("%d\n",FIFO_FLAG);
	//for(j=0;j<3;j++){
	//startpoint = popBlock(A,targetBlock);
	//targetNode = popNode(A,targetBlock);
	//startpoint = targetNode->startpoint;
	if (i==0) {
		xl_valx=0;xl_valy=0;xl_valz=0;
		gy_valx=0;gy_valy=0;gy_valz=0;
		startpoint = (int)popNode(A, targetBlock);
		printf("Popping Node, SP: %d\n", startpoint);
	}
	//targetBlock = A->head;//blockFIFO+(FIFO_FLAG-1)*sizeof(char)*32;
	//printf("start: %d\n",startpoint);
	//while ((i2cReadByteData(pfd,0x3B)&(0x10)) == 0){
	//fifo_num = (i2cReadWordData(pfd,0x3a)*2);
	//for(j=0;j<(fifo_num/32);j++){
	//num=i2cReadI2CBlockData(pfd,0x3e,(blockFIFO),32u);
	//printf("%d Read:%d bytes\n",fifo_num,num);
 	//for (i=0; i<16; i++){
	curVal = (int16_t)combineCharsToInt(*(targetBlock+sizeof(char)*((2*i)+1)),*((targetBlock)+sizeof(char)*2*i));
//	curVal = (int16_t)combineCharsToInt(targetBlock[(2*i)+1], targetBlock[(2*i)]);
	//curVal = convertTwosCompToInt(curVal,16);
	//printf("val: %d\n",curVal);	
	switch(startpoint) {
		case 0:
		case 6:
		case 9:
		case 12:
			gy_valx += curVal*dt*LSM6DS3_DPS/(float)32767 - (offset_gy_x/(float)4);
			break;
		case 1:
		case 7:
		case 10:
		case 13:
			gy_valy += curVal*dt*LSM6DS3_DPS/(float)32767 - offset_gy_y/(float)4;
			break;
		case 2:
		case 8:
		case 11:
		case 14:
			gy_valz += curVal*dt*LSM6DS3_DPS/(float)32767 - offset_gy_z/(float)4;
			break;
		case 3:
			xl_valx = curVal*LSM6DS3_XLG/(float)32767 - offset_xl_x;
			break;
		case 4:
			xl_valy = curVal*LSM6DS3_XLG/(float)32767 - offset_xl_y;
			break;
		case 5:
			xl_valz = curVal*LSM6DS3_XLG/(float)32767 - offset_xl_z;
			break;
		default:
			printf("FIFO ERROR\n");
			break;
	}
	if (startpoint < 14)
		startpoint++;
	else 
		startpoint=0;
	//}
	if (i == 15) {
		i=0;
		avg_xl_valx = xl_valx;///(float)j;///(NUM_SAMPLES/15);
		avg_xl_valy = xl_valy;///(float)j;///(float)(NUM_SAMPLES/15);
		avg_xl_valz = xl_valz;///(float)j;///(NUM_SAMPLES/15);
		if (CAL_MODE) {
			printf("CALIBRATION COMPLETE\n");
			printf("========================\n");
			//delay(100);
			CAL_MODE=0;
			offset_gy_x = gy_valx;
			offset_gy_y = gy_valy;
			offset_gy_z = gy_valz;
			offset_xl_x = xl_valx;
			offset_xl_y = xl_valy;
			offset_xl_z = (float)1.0-xl_valz;
		        printf("Gyro X Offset: %0.4f\n", offset_gy_x);
			printf("Gyro Y Offset: %0.4f\n", offset_gy_y);
			printf("Acc Y Offset: %0.4f\n", offset_xl_y);
			printf("Acc Z Offset: %0.4f\n", offset_xl_z);
		} else {
			printf("x-gy: %0.4f\n", gy_valx);
			printf("z-gy: %0.4f\n", gy_valz);
			printf("x-acc: %0.4f\n", avg_xl_valx);
			printf("y-acc: %0.4f\n", avg_xl_valy);
			printf("z-acc: %0.4f\n", avg_xl_valz);
				
			angle_x=0.97*(angle_x+gy_valx)+(0.03)*(atan2f(avg_xl_valy,avg_xl_valz) * 180 / PI);
			angle_y=0.97*(angle_y-gy_valy)+(0.03)*(atan2f(avg_xl_valx,(avg_xl_valz)) * 180 / PI);	
			angle_z=(angle_z+gy_valz);//+(0.03)*(atan2f(avg_xl_valx, avg_xl_valy) * 180 / PI);
			//if (abs(angle_x) > 15) {
			//	return 0;
			//}
			prev_angle_x = angle_x;
			currAxis[0] = angle_x;
			currAxis[1] = angle_y;
			currAxis[2] = angle_z;
			printf("x-axis: %0.4f\n",angle_x);
			printf("y-axis: %0.4f\n",angle_y);
			printf("z-axis: %0.4f\n",angle_z);	
		}
		FIFO_FLAG--;
	} else {
		i++;
	}
        //gy_valx = gy_valx-offset_gy_x;
        //fromFIFO = (((*blockFIFO)&(0xff))<<8)|( *(blockFIFO+sizeof(char)) );
        //fromFIFO = convertTwosCompToInt(fromFIFO,16);
	//printf("block x-gy: %d\n",fromFIFO);
        //}
	//}
        //printf("DONE\n");
        //free(&targetBlock);
        }
	//delayMicroseconds(1001);
	}
    freeLinkedList(&A); 
    return 0;
}

