#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include "LinkedListStruct.h"
#include <time.h>
#include <signal.h>
#include <pigpio.h>
#include <unistd.h>
#include <pthread.h>
#include "LSM6DS3_Collection.h"
#include "i2c-dev.h"
#include "IMU.h"
#include <fcntl.h>
#define dt104	0.009615384615
#define dt52	0.01923077
#define dt416	0.00240384615
#define dt833	0.00120048019
#define dt1660	0.00060240963
#define dt3330	0.000300030
#define dt	dt1660

// LSM6DS3 CONFIGURATION
// --------------------------------------------------------------------------
#define LSM6DS3_DEVID 0x6b
#define LSM6DS3_DPS 500
#define LSM6DS3_XLG 2

#define LPF_A	0.375
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


// SAMPLING PARAMETERS
// ------------------------------------

#define XL_SAMPLE_COUNT		25
#define CALIB_SAMPLE_COUNT	1000			
uint32_t *BLOCK_READY;
char EXIT_FLAG=0;    
int DRDY_FLAG=0;
int TAP_FLAG=0;
int IN_ISR=0;
char startpoint=0;
uint8_t FIFO_INT_STATUS;
float currAxis[3];
FILE *fp;
uint8_t FIFO_FLAG;  
char *rxBufferPing;
char *rxBufferPong;
int pfd;
extern struct AbsolutePositionStruct *AbsolutePosition;
extern struct pCollection_args *gDeltaArgs;
struct timespec tNow,tLast;
uint16_t deltaT;
/*struct pCollection_args {
	uint8_t BLOCK_READY;
	float x;
	float y;
	float z;
};

struct pCollection_args *args;
*/

uint8_t bufferState=0;
uint8_t txBufferState=0;
void DRDY_ISR(void){
	clock_gettime(CLOCK_MONOTONIC_RAW, &tLast);
	fprintf(fp,"enter isr\nrxbufferstate: %d\n",bufferState);
	if (bufferState){
		i2cReadI2CBlockData(pfd,0x3E,rxBufferPing,30u);
//		i2cReadI2CBlockData(pfd,0x3E,(rxBufferPing+(sizeof(char)*30)),30u);
	} else {
		i2cReadI2CBlockData(pfd,0x3E,rxBufferPong,30u);
//		i2cReadI2CBlockData(pfd,0x3E,(rxBufferPong+(sizeof(char)*30)),30u);
	}
    	i2cWriteByteData(pfd,0x0a,0b01010000);
    	i2cWriteByteData(pfd,0x0a,0b01010001);
	//clock_gettime(CLOCK_MONOTONIC_RAW, &tNow);
//	deltaT = (uint64_t)(tNow.tv_nsec - tLast.tv_nsec);
	bufferState = !(bufferState);
	++FIFO_FLAG;
	//fprintf(fp,"status: %d\n",(0x0FFF&i2cReadWordData(pfd,0x3A)));
//	fprintf(fp,"FIFO_FLAG: %d\n",FIFO_FLAG);
	gArgs->BLOCK_READY=1;
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
	fifo_num = (i2cReadWordData(pfd,0x3A)&(0x0FFF));
	printf("Emptying FIFO of %d bytes ",fifo_num);
	while ((i2cReadByteData(pfd,0x3B)&(0x10)) == 0) {
		printf(".");
		i2cReadWordData(pfd,0x3e);//,fillBuff,32u);
	}
    	i2cWriteByteData(pfd,0x0a,0b01010000);
    	//i2cWriteByteData(pfd,0x0a,0b01010001);
	//fillBuff=NULL;
	printf("Done\n");
}  

/*int LSM6DS3_SetupSPI(){
	int spi_fd = spiOpen(0,50000000,0);
	if (spiRead(spi_fd, 0x0F, 1u) != 0x69){
		printf("IMU not detected\n");

	    	gArgs->CAL_COMPLETE=1;
	    	pthread_exit(NULL);
	} else {
		printf("Found IMU\n");
	return spi_fd;
}*/
void LSM6DS3_Setup(int pfd){
    // check device id
    if (i2cReadByteData(pfd,0x0F) != 0x69){
	    printf("IMU not detected\n");
	    gArgs->CAL_COMPLETE=1;
	    pthread_exit(NULL);
    }
    i2cWriteByteData(pfd,0x12,0x44);
    i2cWriteByteData(pfd,0x0d,0x00);
    i2cWriteByteData(pfd,0x0e,0x20);//0x08); 
    //i2cWriteByteData(pfd,0             x06,0xDC); //set FTH
    i2cWriteByteData(pfd,0x06,0x10);//211;
    i2cWriteByteData(pfd,0x13,0x01);
    i2cWriteByteData(pfd,0x0a,0b01010001);
    // LPF SETUP
    //
    LSM6DS3_EmptyFIFO(pfd);
    
    i2cWriteByteData(pfd,0x0a,0b01010000);
    FIFO_FLAG=0;
    i2cWriteByteData(pfd,0x17,0b00000000); 
    i2cWriteByteData(pfd,0x11,0b10000100); //1.66kHz FS=245dps
    i2cWriteByteData(pfd,0x10,0b10100000);
    i2cWriteByteData(pfd,0x08,0b00100001); //gyro no decimation, xl decimation 4
    delayMicroseconds(20);
    i2cWriteByteData(pfd,0x07,0x00);
    i2cWriteByteData(pfd,0x0a,0b01010001);// ODR=1.66kHz, FIFO MODE
    //delayMicroseconds(20);
    //i2cWriteByteData(pfd,0x0b,0x00);
    //fprintf(fp, "done setup\n");
    /*wiringPiI2CWriteReg8(fd,0x12,0x44);
    wiringPiI2CWriteReg8(fd,0x0d,0x00);
    wiringPiI2CWriteReg8(fd,0x0e,0x08); 
    wiringPiI2CWriteReg8(fd,0x0a,0x26); 
    LSM6DS3_EmptyFIFO(pfd);
    wiringPiI2CWriteReg8(fd,0x0a,0x30);   wiringPiI2CWriteReg8(fd,0x11,0b01100100);
    wiringPiI2CWriteReg8(fd,0x11,0b01100100);
    wiringPiI2CWriteReg8(fd,0x10,0b01000000);
    wiringPiI2CWriteReg8(fd,0x08,0b00001100); //gyro no decimation, xl decimation 4
    wiringPiI2CWriteReg8(fd,0x07,0x00);
    wiringPiI2CWriteReg8(fd,0x06,0x90); //set FTH
    wiringPiI2CWriteReg8(fd,0x0a,0b00110110);// ODR=416, CONTINUOUS MODE*/
    printf("Running IMU Calibration\n------------------------\n");
}

void stop(int signum) {
    printf("Freeing buffers...\n");
    EXIT_FLAG=1; 
}

void *runCollection(void *bp){
    struct pCollection_args *pArgs = (struct pCollection_args*)bp;
    int num;
    int fd;
    struct KalmanFilter *KalmanAxisX;
    struct KalmanFilter *KalmanAxisY;
    fp=fopen("out.log","w");
    fprintf(fp, "New log file from LSM6DS3 Collection\n");
    LinkedList A;
    A = newLinkedList();
    double xl_valx,xl_valy,xl_valz;
    double gy_valx=0,gy_valy=0,gy_valz=0;
    double avg_xl_valx=0,avg_xl_valy=0,avg_xl_valz=1.0;
    double sum_xl_valx=0, sum_xl_valy=0, sum_xl_valz=0;   
    double angle_x=0,angle_y=0,angle_z=0;
    
    if(wiringPiISR(16,INT_EDGE_RISING,&DRDY_ISR) < 0) printf("Error\n");


    int i=0;
    uint8_t k;
    char *txBlock;
    //rxBlockNew = malloc(sizeof(char)*420);
    rxBufferPing = malloc(sizeof(char)*30);
    rxBufferPong = malloc(sizeof(char)*30);
    pfd = i2cOpen(1,LSM6DS3_DEVID,0);
    if (pfd < 0){
	    perror("pigpio i2c failed");
	    exit(1);
    }
    if ((fd=wiringPiI2CSetup(LSM6DS3_DEVID))<0) printf("Failed");
    if (fd < 0){
	perror("Wiring Pi open fail");
	exit(1); 
    }   
    float prev_xl_valz = 1;    
    double offset_gy_x=0,offset_gy_y=0,offset_gy_z=0;
    double offset_xl_x=0;
    double offset_xl_y=0;
    double offset_xl_z=0.0;
    int16_t curVal;
    int testVal=0;
    startpoint = -1;
    emptyList(A);  
    char CAL_MODE=1;
    char EMPTY_FIFO=0; 
    double kalAngleX =0, kalAngleY =0;
    int prev_angle_x;    
    uint8_t j;
    int ping=0;		
    //uint16_t k=0;
    uint16_t fnum;
    uint16_t fifoStatus;
    char thisStart;
    uint8_t REALIGN_FLAG=0;	
    float vx = 0,vy=0, vz=0;
    char devblock[80];
    double sum_OX=0,sum_OY=0,sum_OZ=0;
    int result;
    struct timespec last;
    uint16_t cal_count=0;
    uint8_t xl_count=0;
    uint64_t tDelta=0;
    double initAngleX=0;
    double initAngleY=0;
    /*char lsmfilename[20];
    sprintf(lsmfilename, "/dev/i2c-1");
    char bkfd;
    bkfd = open(lsmfilename,O_RDWR);
    if (bkfd<0){
	    printf("Unable to open i2c bus\n");
    }*/
    // -----------------------------
    // START MAIN LOOP
    // -----------------------------
    //LSM6DS3_SetupSPI();
    LSM6DS3_Setup(pfd);
    double meas_angle_x;
    double LPF_xl_valx=0.0;
    double xl_valx_last=0;
    double xl_valy_last=0;
    double xl_valz_last=1.0;	
    double meas_angle_y;
    txBufferState=!bufferState;
    double accDataX[4]; // 4 datapoints per blcok
    double accDataY[4];
    double accDataZ[4];
    double accVecSquared[4];
    double accDataLPF[3] = {0.0, 0.0, -1.0};
    double accAngle[2] = {0.0, 0.0};
    double sum_gy_valx=0,sum_gy_valy=0,sum_gy_valz=0;
    double accSumDataZ=0, accSumDataX=0, accSumDataY=0;
    uint32_t sampCount=0;
    struct timespec tSampLast, tSampNow;
    clock_gettime(CLOCK_MONOTONIC_RAW, &tSampLast);
    tLast = tSampLast;
    while( (1) ){//&& (EXIT_FLAG==0)) {
	if (FIFO_FLAG>0) {
		if (i==0) {
			fprintf(fp, "FIFO_FLAG: %d\n",FIFO_FLAG);
			//tDelta = (tNow.tv_nsec-tLast.tv_nsec)/1000.0;
			//fprintf(fp, "deltaT: %0.4f\n", (deltaT/(1000000000.0*dt)));
			xl_valx=0;xl_valy=0;xl_valz=0;
			if (!CAL_MODE){
				gy_valx=0;gy_valy=0;gy_valz=0;
			}
			if (!bufferState){
				txBlock = rxBufferPing;
			} else {
				txBlock = rxBufferPong;
			}
			//txBufferState = !txBufferState;
	//		txBlock = (rxBlockNew+(sizeof(char)*(30*(14-FIFO_FLAG))));
			startpoint=0;
			//startpoint = (int)popNode(A, txBlock);
			//fprintf(fp,"Popping Node, SP: %d\n", startpoint);
		
		}
		
		curVal = (int16_t)combineCharsToInt(*(txBlock+(sizeof(char)*((2*i)+1)) ),*(txBlock+(sizeof(char)*2*i)) );
		//fprintf(fp,"txH: %d --- txL: %d\n", *(txBlock+(sizeof(char)*((2*i)+1)) ), *((txBlock)+(sizeof(char)*2*i)) );	
		//fprintf(fp, "startpoint: %d\n", startpoint);4
		if ( (startpoint >= 6) && (startpoint <= 8) ){
		;//	curVal = curVal * 2;
		}
		switch(startpoint) {
			case 0:
				fprintf(fp, "curVal gyx: %d\n",curVal);
				gy_valx = -curVal;
				break;
			case 1:
				gy_valy = -curVal;
				break;
			case 2:
				gy_valz = curVal;
				break;
//				curVal = curVal*(1.0+(deltaT/(1.0e9*dt)));
			case 3:
			case 6:
			case 9:
			case 12:
				accDataX[(startpoint/3)-1] = -curVal*(LSM6DS3_XLG/32768.0)-offset_xl_x;
				//xl_valx += curVal;
				break;
			case 4:
			case 7:
			case 10:
			case 13:
				//xl_valy += curVal*LSM6DS3_DPS*dt/(double)32768.0 - offset_gy_y;///(double)4;
				accDataY[((startpoint-1)/3)-1] = curVal*(LSM6DS3_XLG/32768.0)-offset_xl_y;
				break;
			case 5:
			case 8:
			case 11:
			case 14:
				accDataZ[((startpoint-2)/3)-1] = -curVal*(LSM6DS3_XLG/32768.0)+offset_xl_z;
				//xl_valz += curVal*LSM6DS3_DPS*dt/(double)32768.0 - offset_gy_z;///(double)4;
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
			//fprintf(fp,"end block\n");
			if (CAL_MODE) {
				if ((cal_count % 10)==0){
					printf(".");
					fflush(stdout);
				}
				sum_gy_valx += gy_valx;
				sum_gy_valy += gy_valy;
				sum_gy_valz += gy_valz;
				for(j=0;j<4;j++){
					accSumDataX += accDataX[j];
					accSumDataY += accDataY[j];
					accSumDataZ += accDataZ[j];	
				}
				
				/*sum_xl_valx += xl_valx;
				sum_xl_valy += xl_valy;
				sum_xl_valz += xl_valz;*/
				cal_count++;
				xl_valx=0;xl_valy=0;xl_valz=0;
				if (cal_count == CALIB_SAMPLE_COUNT){
					printf("\nCALIBRATION COMPLETE\n");
					printf("========================\n");
					CAL_MODE=0;
					pArgs->CAL_COMPLETE=1;
					offset_gy_x = sum_gy_valx*(LSM6DS3_DPS/(CALIB_SAMPLE_COUNT*32768.0));//16000.0;
					offset_gy_y = sum_gy_valy*(LSM6DS3_DPS/(CALIB_SAMPLE_COUNT*32768.0));//16000.0;
					offset_gy_z = sum_gy_valz/(double)(CALIB_SAMPLE_COUNT);//16000.0;
					offset_xl_x = accSumDataX/(double)CALIB_SAMPLE_COUNT/4.0;//4000.0;
					offset_xl_y = accSumDataY/(double)CALIB_SAMPLE_COUNT/4.0;//4000.0;
					offset_xl_z = -1.0-(accSumDataZ/CALIB_SAMPLE_COUNT/4.0);//0;//1.0-((sum_xl_valz/(double)CALIB_SAMPLE_COUNT)*(2.0/(32768.0*4.0)));//0.00006103515625);
					avg_xl_valx = 0;//sum_xl_valx/((double)CALIB_SAMPLE_COUNT*4.0*32768.0)*LSM6DS3_XLG;
					avg_xl_valy = 0;//sum_xl_valy/((double)CALIB_SAMPLE_COUNT*4.0*32768.0)*LSM6DS3_XLG;
					initAngleX = 0;//(180 / M_PI)*atan2(avg_xl_valy,avg_xl_valz);
					initAngleY = 0;//(180 / M_PI)*atan2(-avg_xl_valx,avg_xl_valz);
					meas_angle_x = initAngleX;

					fprintf(fp,"Gyro X Offset: %0.4f\n", offset_gy_x);
					fprintf(fp,"Gyro X Offset: %0.4f\n", offset_gy_x);
					fprintf(fp,"Init Angle X: %0.4f\n", initAngleX);
					fprintf(fp,"Init Angle Y: %0.4f\n", initAngleY);
					fprintf(fp,"Acc X Offset: %0.4f\n", offset_xl_x);
					fprintf(fp,"Acc Y Offset: %0.4f\n", offset_xl_y);
					fprintf(fp,"Acc Z Offset: %0.4f\n\n%0.4f\n", offset_xl_z, (1.0+offset_xl_z));
					KalmanAxisX = NewKalmanFilter(initAngleX,offset_gy_x);
					KalmanAxisY = NewKalmanFilter(initAngleY,offset_gy_y);
					accDataLPF[0] = 0.0;
					accDataLPF[1] = 0.0;
					accDataLPF[2] = -1.0;
					sum_xl_valx=0;sum_xl_valy=0;sum_xl_valz=0;
				//	clock_gettime(CLOCK_MONOTONIC_RAW,&tLast);
				//}
				}
			} else {
				fprintf(fp, "BLOCK DONE\n");
				clock_gettime(CLOCK_MONOTONIC_RAW,&tNow);
				deltaT = (uint64_t)(tNow.tv_nsec - tLast.tv_nsec);
				for(j=0;j<4;j++){
					accDataLPF[0] += LPF_A * (accDataX[j]-accDataLPF[0]);
					accDataLPF[1] += LPF_A * (accDataY[j]-accDataLPF[1]);
					accDataLPF[2] += LPF_A * (accDataZ[j]-accDataLPF[2]);
//					accVecSquared[j] = accDataX[j]*accDataX[j]+accDataY[j]*accDataY[j]+accDataZ[j]*accDataZ[j];
					
					//if ((accVecSquared[j] < 1.2) && (accVecSquared[j] > 0.85)){
						//for (k=0;k<3;k++) {
						//	accDataLPF[k] += LPF_A*(accDataX[j]-accDataLPF[k]);}
					//} else {

						//printf("\n@@@@@@@ ignored reading @@@@@@@@@\n");
					//}
				}
				double accMagSquared=0;
				for(j=0;j<3;j++) { accMagSquared += accDataLPF[j]*accDataLPF[j]; }
				if( (accMagSquared < 1.26) && (accMagSquared > 0.85) ){

					accAngle[0] = (180.0/M_PI)*atan2(accDataLPF[1],-accDataLPF[2]);
					accAngle[1] = 180.0/M_PI*(atan(-accDataLPF[0] / (sqrt(accDataLPF[1]* accDataLPF[1] + accDataLPF[2] * accDataLPF[2]))));
				}
				prev_xl_valz=xl_valz;
				LPF_xl_valx = LPF_xl_valx + LPF_A*(xl_valx-LPF_xl_valx);
				avg_xl_valx = (LPF_xl_valx-(offset_xl_x))*2.0/(4.0*32768.0);
				avg_xl_valy = -(xl_valy-(offset_xl_y))*2.0/(4.0*32768.0);//0.00000244140625;
				avg_xl_valz = -((xl_valz*2.0/(4.0*32768.0))+offset_xl_z);
				gy_valz = (gy_valz-offset_gy_z)*(3*dt*500.0/(32768.0));//9.5467431650625e-6;///108789.76;	
				gy_valx = (gy_valx)*(500.0/(32768.0));
				gy_valy = (gy_valy)*(500.0/(32768.0));
				angle_z += gy_valz;
				
				/*if ((avg_xl_valy < 1.3) && (avg_xl_valy > -0.3)){
					meas_angle_x = (180.0/M_PI)*atan2(avg_xl_valy,avg_xl_valz);////angle_x + (gy_valx*4*dt);
				} else {
					meas_angle_x = kalAngleX;
				}*/
				//double meas_angle_x = 180.0/M_PI*(atan(avg_xl_valy / (sqrt(avg_xl_valx * avg_xl_valx + avg_xl_valz * avg_xl_valz))));
				meas_angle_x = (180.0/M_PI)*atan2(avg_xl_valy,-avg_xl_valz);////angle_x + (gy_valx*4*dt);
				meas_angle_y = 180.0/M_PI*(atan(avg_xl_valx / (sqrt(avg_xl_valy * avg_xl_valy + avg_xl_valz * avg_xl_valz))));
				
				if ( ((meas_angle_x < -90) && (kalAngleX > 90)) || ((meas_angle_x > 90) && (kalAngleX < -90)) ){
					kalAngleX = accAngle[0];
				} else {	       
					kalAngleX = getAngleKalman(KalmanAxisX,accAngle[0],gy_valx,2.875*dt);
				}
				if (abs(kalAngleX > 90)){
					gy_valy = -gy_valy;
				}
				kalAngleY = getAngleKalman(KalmanAxisY,accAngle[1],gy_valy,2.875*dt); 
				//tLast=tNow;
				//printf("tdelta: %ld\n", deltaT);
				angle_x += (gy_valx-offset_gy_x)*2.875*dt;
				angle_y += gy_valy*3.0*dt;
				//kalAngleXLast = kalAngleX;
				//fprintf(fp, "%0.3f, %0.3f\n", angle_x, kalAngleX);
				//angle_y=0.98*(angle_y-gy_valy)+(0.02)*(atan2(avg_xl_valx,avg_xl_valz) * 180 / M_PI);	
				//angle_x = angle_x + kalAngle;
				pArgs->x = kalAngleX;
				pArgs->y = kalAngleY;
				pArgs->z = angle_z;
				xl_count++;
	//			sum_xl_valx+=xl_valx;
	//			sum_xl_valy+=xl_valy;
	//			sum_xl_valz+=xl_valz;
				xl_valx=0;
				xl_valy=0;
				xl_valz=0;
				pArgs->xl_x = accDataLPF[0];//avg_xl_valx;
				pArgs->xl_y = accDataLPF[1];//avg_xl_valy;//avg_xl_valy;
				pArgs->xl_z = accDataLPF[2];//avg_xl_valz;//avg_xl_valz;//xl_valz;
				sum_xl_valx+=avg_xl_valx;
				sum_xl_valy+=avg_xl_valy;
				sum_xl_valz+=avg_xl_valz;
		//		OrdinalAxis->X = avg_xl_valx;
		//		OrdinalAxis->Y = avg_xl_valy;
		//		OrdinalAxis->Z = avg_xl_valz;
				if (xl_count == 1){
					gDeltaArgs->x = kalAngleX - sum_OX; // ** Note: Not actually Sum variables .. too lazy to declare new ones
					gDeltaArgs->y = kalAngleY - sum_OY;
					gDeltaArgs->z = angle_z - sum_OZ;
					OrdinalAxis->X = accDataLPF[0];
					OrdinalAxis->Y = accDataLPF[1];
					OrdinalAxis->Z = accDataLPF[2];
					//OrdinalAxis->X=0.0;//sum_xl_valx/100.0;
					//OrdinalAxis->Y=0.0;//sum_xl_valy/100.0;
					//OrdinalAxis->Z=1.0;//sum_xl_valz/100.0;
					sum_OX = kalAngleX;
					sum_OY = kalAngleY;
					sum_OZ = angle_z;
					//struct CoordinateAxis tempAxis = OrdinalAxis;
					//updateXLCoordinateAxis(*pArgs,OrdinalAxis);
					rotateBtoI(*pArgs,OrdinalAxis);
					//sum_OX += OrdinalAxis->X;
					//sum_OY += OrdinalAxis->Y;
					//sum_OZ += OrdinalAxis->Z;
					vx += (OrdinalAxis->X)*(981.0*3.0*dt); //(float)(deltaT*1.0e-09);//*0.00721*3.0;
					vy += OrdinalAxis->Y*981.0*(3.0*dt);//*3.0;
					vz += (OrdinalAxis->Z+1.0)*(981.0*3.0*dt);//981.0*0.06;//
					AbsolutePosition->vX = angle_x;
					sum_xl_valx=0;
					sum_xl_valy=0;
					sum_xl_valz=0;
					//sum_OX = 0;sum_OY=0;sum_OZ=0;	
					xl_count=0;
				}
		//		avg_xl_valx = -1.0*(sum_xl_valx-(offset_xl_x*25.0))*2.0/(25.0*4.0*32768.0);
		//		avg_xl_valy = (sum_xl_valy-(offset_xl_y*25.0))*2.0/(25.0*4.0*32768.0);//0.00000244140625;
		//		avg_xl_valz = (sum_xl_valz*2.0/(25*4.0*32768.0))+offset_xl_z;
//				sum_xl_valx=0;
//				sum_xl_valy=0;
//				sum_xl_valz=0;
				//}
				gy_valx=0;gy_valy=0;gy_valz=0;	
				clock_gettime(CLOCK_MONOTONIC_RAW, &tSampNow);
				if ( (uint64_t)(tSampNow.tv_nsec-tSampLast.tv_nsec) >= 5e7){
					tSampLast = tSampNow;
					fprintf(fp, "%d, %0.3f, %0.3f, %0.3f\n",sampCount, angle_x, meas_angle_x, kalAngleX);
					sampCount++;
				}
				prev_angle_x = angle_x;
				/*fprintf(fp, "x-axis: %0.4f\n",pArgs->x);
				fprintf(fp, "y-axis: %0.4f\n",angle_y);
				fprintf(fp, "z-axis: %0.4f\n",angle_z);*/
				//}
			}
			FIFO_FLAG--;
		} else {
			i++;
		}
	}
    }
    free(rxBufferPing);
    free(rxBufferPong);
    free(txBlock);
    fclose(fp);
    freeLinkedList(&A); 
    pthread_exit(NULL);
}

