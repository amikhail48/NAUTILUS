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
#include <pthread.h>
#include "LSM6DS3_Collection.h"
#define PI	3.14159265359
#define dt104	0.009615384615
#define dt52	0.01923077
#define dt416	0.00240384615
#define dt833	0.00120048019
#define dt1660	0.00060240963
#define dt	dt1660

// LSM6DS3 CONFIGURATION
// --------------------------------------------------------------------------
#define LSM6DS3_DEVID 0x6b
#define LSM6DS3_DPS 245 
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

uint32_t *BLOCK_READY;
char EXIT_FLAG=0;    
int DRDY_FLAG=0;
int TAP_FLAG=0;
int IN_ISR=0;
char startpoint=0;
uint8_t FIFO_INT_STATUS;
float currAxis[3];

/*struct pCollection_args {
	uint8_t BLOCK_READY;
	float x;
	float y;
	float z;
};

struct pCollection_args *args;
*/
void DRDY_ISR(void){
	//printf("enter isr\n");
	args->BLOCK_READY=1;
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

void LSM6DS3_EmptyFIFO(int pfd){
	uint16_t fifo_num;
	//uint8_t *fillBuff;
	if ((i2cReadByteData(pfd,0x3B)&0x70)){
		fifo_num = 4096;
	} else {	
		fifo_num = (i2cReadWordData(pfd,0x3A)&(0x0FFF));
	}
	printf("Emptying FIFO of %d bytes ",fifo_num);
	while ((i2cReadByteData(pfd,0x3B)&(0x10)) == 0) {
		printf(".");
		i2cReadWordData(pfd,0x3e);//,fillBuff,32u);
	}
	//fillBuff=NULL;
	printf("Done\n");
}    
void LSM6DS3_Setup(int pfd){
    i2cWriteByteData(pfd,0x12,0x44);
    i2cWriteByteData(pfd,0x0d,0x00);
    i2cWriteByteData(pfd,0x0e,0x20);//0x08); 
    //i2cWriteByteData(pfd,0x06,0xDC); //set FTH
    i2cWriteWordData(pfd,0x06,0x005A); //5DC
    i2cWriteByteData(pfd,0x13,0x01);
    i2cWriteByteData(pfd,0x0a,0x41);
    
    LSM6DS3_EmptyFIFO(pfd);
    i2cWriteByteData(pfd,0x0a,0x40);
    i2cWriteByteData(pfd,0x11,0b10000000);
    i2cWriteByteData(pfd,0x10,0b01100000);
    i2cWriteByteData(pfd,0x08,0b00001100); //gyro no decimation, xl decimation 4
    i2cWriteByteData(pfd,0x07,0x00);
    i2cWriteByteData(pfd,0x0a,0b01000001);// ODR=416, FIFO MODE
}

void stop(int signum) {
    printf("Freeing buffers...\n");
    EXIT_FLAG=1; 
}

void *runCollection(void *bp){
    struct pCollection_args *pArgs = (struct pCollection_args*)bp;
    int num;
    int fd;
    int pfd;
    FILE *fp;
    fp=fopen("out.log","w");
    fprintf(fp, "New log file from LSM6DS3 Collection\n");
    LinkedList A;
    A = newLinkedList();
    float xl_valx,xl_valy,xl_valz;
    float gy_valx,gy_valy,gy_valz;
    //float avg_xl_valx, avg_xl_valy, avg_xl_valz;
    float angle_x=0,angle_y=0,angle_z=0;
    
    if(wiringPiISR(16,INT_EDGE_RISING,&DRDY_ISR) < 0) printf("Error\n");


    int i=0;
    char *rxBlock;
    char *txBlock;
    rxBlock = malloc(sizeof(char)*30);
    txBlock = malloc(sizeof(char)*30);
    pfd = i2cOpen(1,0x6b,0);
    if ((fd=wiringPiI2CSetup(LSM6DS3_DEVID))<0) printf("Failed");
    if (fd < 0){
	perror("Wiring Pi open fail");
	exit(1); 
    }   
    LSM6DS3_Setup(pfd);
    float prev_xl_valz = 1;    
    float offset_gy_x=0,offset_gy_y=0,offset_gy_z=0;
    float offset_xl_x=0;
    float offset_xl_y=0;
    float offset_xl_z=0;
    int curVal;
    int testVal=0;
    startpoint = -1;
    uint32_t FIFO_FLAG=0;
    emptyList(A);  
    char CAL_MODE=1;
    char EMPTY_FIFO=0; 
    int prev_angle_x;    
    int j;
     		
    uint16_t k=0;
    uint16_t fnum;
    uint16_t fifoStatus;
    char thisStart;
    uint8_t REALIGN_FLAG=0;	
    // -----------------------------
    // START MAIN LOOP
    // -----------------------------
    while( (1) ){//&& (EXIT_FLAG==0)) {
	if (pArgs->BLOCK_READY==1) {
		if (REALIGN_FLAG == 1){
			//LSM6DS3_EmptyFIFO(pfd);
			REALIGN_FLAG=0;
			emptyList(A);
			thisStart =0;
			i=0;
			FIFO_FLAG=0;
			//startpoint = thisStart;
			fprintf(fp,"New startpoint: %d\n",thisStart);
		}
		//pArgs->BLOCK_READY=0;
		k=0;
		fifoStatus = (uint16_t)i2cReadWordData(pfd,0x3A);
		fprintf(fp,"status: %x\n", ((fifoStatus)&(0x6000)));
		//num = (fifoStatus)&(0x0FFF);
		if ( ((fifoStatus)&(0x6000))==  0 )
			fnum = (fifoStatus)&(0x0FFF);
		else {
			//printf("FIFO OVRFLOW\n");
			//fprintf(fp,"FIFO OVRFLOW\n");
                        i2cReadI2CBlockData(pfd,0x3E, (rxBlock),30u);
			//while((i2cReadByteData(pfd,0x3B)&(0x10))==0){
                                //printf("Reading\n");
				i2cReadI2CBlockData(pfd,0x3E, (rxBlock),30u);
				addNode(A,rxBlock, thisStart);
				FIFO_FLAG++;
			//} 
			fnum = 0;//(uint16_t)((i2cReadWordData(pfd,0x3A))&(0x0FFF));
		}
		pArgs->BLOCK_READY=0;
		fprintf(fp,"Done - fnum: %d\n",fnum);
	}
	
	if ( (FIFO_FLAG>0) && (REALIGN_FLAG ==0 )){
		if (i==0) {
			fprintf(fp, "new block\n");
			xl_valx=0;xl_valy=0;xl_valz=0;
			gy_valx=0;gy_valy=0;gy_valz=0;
			startpoint = (int)popNode(A, txBlock);
			fprintf(fp,"Popping Node, SP: %d\n", startpoint);
		}
		curVal = (int16_t)combineCharsToInt(*(txBlock+sizeof(char)*((2*i)+1)),*((txBlock)+sizeof(char)*2*i));
		fprintf(fp, "curval: %d\n", curVal);
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
		if (i == 14) { // End of complete FIFO pattern
			i=0; // reset
			// no longer using average b/c data is summed every pattern loop
			/*avg_xl_valx = xl_valx;///(float)j;///(NUM_SAMPLES/15);
			avg_xl_valy = xl_valy;///(float)j;///(float)(NUM_SAMPLES/15);
			avg_xl_valz = xl_valz;///(float)j;///(NUM_SAMPLES/15);
			*/
			if (CAL_MODE) {
				printf("CALIBRATION COMPLETE\n");
				printf("========================\n");
				CAL_MODE=0;
				offset_gy_x = gy_valx;
				offset_gy_y = gy_valy;
				offset_gy_z = gy_valz;
				offset_xl_x = xl_valx;
				offset_xl_y = xl_valy;
				offset_xl_z = (float)1.0-xl_valz;
				fprintf(fp,"Gyro X Offset: %0.4f\n", offset_gy_x);
				fprintf(fp,"Gyro Y Offset: %0.4f\n", offset_gy_y);
				fprintf(fp,"Acc Y Offset: %0.4f\n", offset_xl_y);
				fprintf(fp,"Acc Z Offset: %0.4f\n", offset_xl_z);
			} else {
				// for debugging
				fprintf(fp,"x-gy: %0.4f\n", gy_valx);
				fprintf(fp,"y-gy: %0.4f\n", gy_valy);
				fprintf(fp,"z-gy: %0.4f\n", gy_valz);
				fprintf(fp,"x-acc: %0.4f\n", xl_valx);
				fprintf(fp,"y-acc: %0.4f\n", xl_valy);
				fprintf(fp,"z-acc: %0.4f\n", xl_valz);
				/*if( abs(prev_xl_valz - xl_valz) > 0.8 ) {
					REALIGN_FLAG=1;
					fprintf(fp,"Trying to re-align\n");
					
					printf("Trying to re-align\n");
				} else {*/
				prev_xl_valz=xl_valz;	
				//angle_x = angle_x+gy_valx;
				angle_x=0.97*(angle_x+gy_valx)+(0.03)*(atan2f(xl_valy,xl_valz) * 180 / PI);
				angle_y=0.97*(angle_y-gy_valy)+(0.03)*(atan2f(xl_valx,(xl_valz)) * 180 / PI);	
				angle_z=(angle_z+gy_valz);//+(0.03)*(atan2f(xl_valx, xl_valy) * 180 / PI);
				pArgs->x = angle_x;
				pArgs->y = angle_y;
				prev_angle_x = angle_x;
				fprintf(fp, "x-axis: %0.4f\n",pArgs->x);
				fprintf(fp, "y-axis: %0.4f\n",angle_y);
				fprintf(fp, "z-axis: %0.4f\n",angle_z);
				//}
			}
			FIFO_FLAG--;
		} else {
			i++;
		}
	}
    }
    free(rxBlock);
    free(txBlock);
    fclose(fp);
    freeLinkedList(&A); 
    pthread_exit(NULL);
}

pthread_t tid[2];
uint32_t *pCollection;
int KP=80;int KI=25;int KD=5;
int main(void) {
	int err;
	args = malloc(sizeof(struct pCollection_args));
	wiringPiSetup();
	gpioInitialise();
        if(wiringPiISR(16,INT_EDGE_RISING,&DRDY_ISR)<0) printf("ERROR\n");
	err = pthread_create(&(tid[0]), NULL, &runCollection, (void*)args);//pCollection);
	if (err != 0) {
		perror("Thread error");
	} else {
		printf("New thread created\n");
	}
	int T_ROLL=0;
	float y_out;
	float y_err_p;
	float y_err_prev;
	float y_err_i=0;
	float y_err_d;
	delay(2000);
	while(1) {
		delay(100);
		//LSM6DS3_GetAxis(axisData);
		//delay(5);
		//printf("READINGS\n========================\n");
		printf("x: %0.4f\n", args->x);//(float)*(pCollection+sizeof(float)));
		printf("y: %0.4f\n", args->y);//(float)*(pCollection+sizeof(float)*2));
		y_err_p = round(args->y-T_ROLL*10)/10;
		y_err_i += y_err_p;
		y_err_d = y_err_prev - y_err_p;
		y_err_prev = y_err_p;
		y_out =	KP*(y_err_p) + KI*(y_err_i) + KD*(y_err_d);
		printf("y-correction: %0.4f\n", y_out);
	}
	free(args);
	//free(pCollection);
	//free(axisData);	
	pthread_exit(NULL);
	//return 0;
}
