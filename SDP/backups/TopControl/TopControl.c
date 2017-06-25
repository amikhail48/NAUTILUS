#include <pigpio.h>
#include <stdlib.h>
#include <stdio.h>
//#include <wiringPi.h>
#include <pthread.h>
#include <math.h>
#include <unistd.h>
#include <time.h>

#include "LSM6DS3_Collection.h"
#include "MS5803_Collection.h"
#include "FeedbackControlTop.h"
#include "receiver.h"
#include "IMU.h"

#include "./EventFramework/EventFramework.h"
#include "./EventFramework/UltrasonicEventChecker.h"
#include "./EventFramework/DepthEventChecker.h"
#include "./EventFramework/HumidityEventChecker.h"
#include "./EventFramework/ADCEventChecker.h"
#include "./EventFramework/eventQueue.h"
#include "./EventFramework/HSM.h"
#include "./EventFramework/MCP3202.h"

pthread_t tid[4];

#define TEST_POSITION_SYSTEM
	
//-------------------- PID PARAMETER DEFS -----------------------
//Adjust KP,KI,KD
#define ROLL_KP	2
#define ROLL_KI	0	
#define	ROLL_KD	1
#define T_ROLL	0	//target tilt

struct pCollection_args *gArgs;
struct pCollection_args *gDeltaArgs;
struct AbsolutePositionStruct	*AbsolutePosition;
struct CoordinateAxis *OrdinalAxis;
struct MotorThrottleStateStruct *MotorThrottleState;
//struct motorControl_args *mArgs;
struct TopFeedbackStateStruct *fbArgs;
//------------ MAIN FUNCTION ----------------------
int main(void) {
	wiringPiSetup();
	gpioInitialise();
	
//	mArgs = malloc(sizeof(struct motorControl_args));	
	psArgs = malloc(sizeof(struct presCollection_args));
	gArgs = malloc(sizeof(struct pCollection_args)); //allocate mem for gyro data - gArgs is updated by runCollection function
	fbArgs = malloc(sizeof(struct TopFeedbackStateStruct));
	AbsolutePosition= malloc(sizeof(struct AbsolutePositionStruct));
	gDeltaArgs = malloc(sizeof(struct pCollection_args));
	MotorThrottleState=malloc(sizeof(struct MotorThrottleStateStruct));
	fbArgs->rollState = malloc(sizeof(struct AxisFeedbackState));
	fbArgs->pitchState = malloc(sizeof(struct AxisFeedbackState));
	fbArgs->yawState = malloc(sizeof(struct AxisFeedbackState));
	fbArgs->VxState = malloc(sizeof(struct AxisFeedbackState));
	fbArgs->zState = malloc(sizeof(struct AxisFeedbackState));
	OrdinalAxis = malloc(sizeof(struct CoordinateAxis));
	const int SPIChan = 0;
	const int SPIFreq = 2000000;
	//MCP3202Setup(ADC_PINBASE,SPIChan,SPIFreq);

	OrdinalAxis->X=0.0;
	OrdinalAxis->Y=0.0;
	OrdinalAxis->Z=-1.0;
	gArgs->CAL_COMPLETE=0;
	uint8_t *tpArgs;
	// Start individual threads
	uint8_t err;
	/*err = pthread_create(&(tid[0]), NULL, &runCollection, (void*)gArgs);
	if (err != 0) {
		perror("IMU thread error");
	} else {
		printf("IMU thread created\n");
	}*/
	err = pthread_create(&(tid[1]), NULL, &receiveCmds, (void*)fbArgs);
	if (err != 0) {
		perror("User input thread error");
	} else {
		printf("User input thread created\n");
	}
	/*err = pthread_create(&(tid[2]), NULL, &runPresCollection, (void*)psArgs);
	if (err != 0) {
		perror("Pressure thread error");
	} else {
		printf("Pressure thread created\n");
	}*/
	float x_out;
	float x_err_p;
	float x_err_prev;
	float x_err_i=0;
	float x_err_d;
	psArgs->depth=0;
	//printf("Waiting for Calibration...\n");
	while (gArgs->CAL_COMPLETE != 1) {;}	
	float prevX=0, prevY=0, prevZ=0;
	fbArgs->rollState->currentValue 	= &(gArgs->x);
	fbArgs->VxState->currentValue 		= &(AbsolutePosition->vX);
	fbArgs->pitchState->currentValue 	= &(gArgs->y);
	fbArgs->yawState->currentValue		= &(gArgs->z);
	fbArgs->zState->currentValue		= &(psArgs->depth);
	
	//fbArgs->rollState->currentValue = 0;//gArgs->x;
	//fbArgs->zState->currentValue = 0;//gArgs->x;
	err = pthread_create(&(tid[3]), NULL, &TopControlLoop, (void*)fbArgs);
	if (err != 0) {
		perror("Feedback Control thread error");
	} else {
		printf("Feedback Control thread created\n");
	}
	//struct pCollection_args deltaAngles;
#ifndef TEST_POSITION_SYSTEM
	Queue hsmQueue = InitHSM();
	time_t timeStart = time(NULL);
	time_t timeNow;
	time_t lastTime = 0;

	//Initialize everything..
	//Start main loop
	int loop = TRUE;
	while (loop){
		delay(200);
        	timeNow = time(NULL);
		int timeRunning = timeNow-timeStart;
		//clock_gettime(CLOCK_REALTIME, &now);
		printf("time: %u\n", timeRunning);
#ifndef USE_KEYBOARD_INPUT
	//Put in wiringPi interrupt
		//Check for new events
		Event event = checkUltrasonicSensors();
		if (event.Type != No_Event){
			PostHSM(event);
	
		}
		printf("Ultrasonic result: %s\n", EventStr[event.Type]);

		/*Event event2 = checkDepthSensor();
		if(event2.Type != No_Event){
			PostHSM(event2);
		}
		printf("Depth result: %s\n", EventStr[event2.Type]);

		if((timeNow - lastTime) > 2){
			Event event3 = checkHumiditySensor();
			if(event3.Type != No_Event){
				PostHSM(event3);
			}
			printf("Humidity result: %s\n\n", EventStr[event3.Type]);
			lastTime = timeNow;
		}

		Event event4 = checkDroneDistance();
		if(event4.Type != No_Event){
			PostHSM(event4);
		}
		printf("Drone Tracking result: %s\n", EventStr[event4.Type]);*/

#endif

		while (queueSize(hsmQueue) > 0){
			//Remove event from queue, and pass it to the HSM
			Event retEvent = RunHSM(removeEvent(hsmQueue));

			if (retEvent.Type != No_Event){
				printf(ANSI_COLOR_RED"Error: Event was not handled in HSM - %s" ANSI_COLOR_RESET "\n", EventStr[retEvent.Type]);
				loop = FALSE;
			}
		}
	}

	// TODO: Need a way to exit while loop
	freeQueue(&hsmQueue);
	pthread_exit(NULL);
	return 0;
#else
	while(1) {
		delay(10);
	//	fbArgs->rollState->currentValue = round(gArgs->x*10)/10.0;
		printf("\rx: %6.3f", gArgs->x);//(float)*(pCollection+sizeof(float)));
		printf(" | y: %6.3f", gArgs->y);//(float)*(pCollection+sizeof(float)*2));
		printf(" | z: %6.3f", gArgs->z);
		printf(" | xl_x: %6.3f",gArgs->xl_x);
		printf(" | xl_y: %6.3f",gArgs->xl_y);
		printf(" | xl_z: %6.3f",gArgs->xl_z);
		printf(" | Vx: %6.3f", AbsolutePosition->vX);
		printf(" | OX: %6.3f",OrdinalAxis->X);
		printf(" | OY: %6.3f",OrdinalAxis->Y);
		printf(" | OZ: %6.3f",OrdinalAxis->Z);
		printf(" | Zset: %5.2f", fbArgs->zState->setpoint);
		fflush(stdout);

	}
	free(gArgs);
	free(psArgs);
	pthread_exit(NULL);
	return 0;
}

#endif
