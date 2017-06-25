
#include "FeedbackMotorController.h"
//#include "receiver.h"
#include <pigpio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#define NEUTRAL_THROTTLE 1500
//#define NUM_MOTORS 5
#define PIN_LTHRUST		19
#define PIN_RTHRUST		16
#define PIN_LTOP		26
#define PIN_RTOP		20
#define PIN_FTOP		21
#define NUM_MOTORS		5


const char MotorPinList[5] = {PIN_LTHRUST, PIN_RTHRUST, PIN_LTOP, PIN_RTOP, PIN_FTOP};
extern struct MotorThrottleStateStruct *MotorThrottleState;

void printMotorThrottleStates(){
	printf("\nLTOP: %5.4f | RTOP %5.4f | FTOP %5.4f \n",
			MotorThrottleState->leftTopMotorThrottle,
			MotorThrottleState->rightTopMotorThrottle,
			MotorThrottleState->frontTopMotorThrottle); 
}
void InitializeMotors(){

	uint8_t i;
	//MotorThrottleState=malloc(sizeof(struct MotorThrottleStateStruct));
	printf("INIT MOTORS\n");	
	for (i=0;i<NUM_MOTORS;i++){
		gpioServo(MotorPinList[i], NEUTRAL_THROTTLE);
	}
	MotorThrottleState->frontTopMotorThrottle=0;
	MotorThrottleState->leftTopMotorThrottle=0;
	MotorThrottleState->rightTopMotorThrottle=0;
	MotorThrottleState->leftThrustMotorThrottle=0;
	MotorThrottleState->rightThrustMotorThrottle=0;
	delay(1000);

}

void updateSingleMotorPW(int pin, int pulse){
	if (pulse > 500)
		pulse = 500;	       
	else if (pulse < -500)
		pulse = -500;
	
	gpioServo(pin, (int)(NEUTRAL_THROTTLE+pulse));
}
/*void updateSingleMotor(int pin, double percent){
	int pulse;
	pulse = NEUTRAL_THROTTLE+(500.0*percent)/100;
	gpioServo(pin,pulse);
}*/
void updateMotorStates(float roll_correction, float pitch_correction, float yaw_correction, float vx_correction, float z_correction, uint8_t isFollowingVertical){
//void updateMotorStates(struct MotorThrottleState *currentThrottleState, struct TopFeedbackState *PIDOutput){
	// Motor A:	Right Top Motor
	// Motor B:	Left Top Motor
	// Motor C:	Front Top Motor

	// roll
	// -------
	// Determined by combined torque of Motors A+B
	//
	
	static int targetThrottleA=0,targetThrottleB=0,targetThrottleC=0;
//	if (1){
	if (!isFollowingVertical){
	//static char currentThrottleA = 0,currentThrottleB=0;
       	//double depthThrottle=0; // Set to 0 for testing, will be determined by % throttle to hold depth
	
		int targetDepthThrottleA = -1.0*z_correction/2.0;
		int targetDepthThrottleB = z_correction/2.0;
		
		if (fabs(roll_correction) > 2.0){
			targetThrottleA = targetDepthThrottleA-roll_correction/2.0;
			targetThrottleB = targetDepthThrottleB-roll_correction/2.0;
		} else {
			targetThrottleA = targetDepthThrottleA;
			targetThrottleB = targetDepthThrottleB;
		}
		if (targetThrottleA > 500)
			targetThrottleA = 500;
		else if (targetThrottleA < -500)
			targetThrottleA = -500;
		
		if (targetThrottleB > 500)
			targetThrottleB = 500;
		else if (targetThrottleB < -500)
			targetThrottleB= -500;
		MotorThrottleState->leftTopMotorThrottle=targetThrottleA;
		MotorThrottleState->rightTopMotorThrottle=targetThrottleB;
	}	
	targetThrottleC = pitch_correction;

//	printf("Updating Motors\n");	
	MotorThrottleState->frontTopMotorThrottle=targetThrottleC;
//	updateSingleMotorPW(PIN_FTOP, 300);//MotorThrottleState->frontTopMotorThrottle);
#ifdef RUN_MOTORS
	updateSingleMotorPW(PIN_LTOP, MotorThrottleState->leftTopMotorThrottle);
	updateSingleMotorPW(PIN_RTOP, MotorThrottleState->rightTopMotorThrottle);//targetThrottleB);
#endif
	//printMotorThrottleStates();

}

