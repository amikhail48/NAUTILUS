// FeedbackControlTop.c
// ----------------------


//#include "autoMotorFunctions.h"
#include "./EventFramework/EventFramework.h"
#include "FeedbackMotorController.h"
#include "FeedbackControlTop.h"
#include <pthread.h>
#include "LSM6DS3_Collection.h"
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <stdbool.h>
//struct MotorThrottleStateStruct *MotorThrottleState;
#define PID_SAMPLE_TIME 100000 //microseconds

extern struct pCollection_args *gArgs;
void initTopPID(struct TopFeedbackStateStruct *PID){
	
	// SET PID PARAMETERS
	// -- TUNE THESE FOR TESTING ---
	PID->rollState->KP	= 15;
	PID->rollState->KI	= 0;
	PID->rollState->KD	= 0;
	
	PID->pitchState->KP	= -27;
	PID->pitchState->KI	= 0;
	PID->pitchState->KD	= 0;
	
	PID->yawState->KP	= 7;
	PID->yawState->KI	= 0;
	PID->yawState->KD	= 1;
	
	PID->VxState->KP	= 85;
	PID->VxState->KI	= 0;
	PID->VxState->KD	= 0;
	
	PID->zState->KP		= 125;
	PID->zState->KI		= 0;
	PID->zState->KD		= 0;
	
	//--------------------------------
	// Initialize all setpoints to 0
	
	PID->rollState->setpoint = 0;
	PID->pitchState->setpoint = 0;
	PID->yawState->setpoint = 0;
	PID->VxState->setpoint = 0;
	PID->zState->setpoint = 0;
	
	// ----------------------------------
	// State variables that tells controller to turn off depth or yaw maintaining. For example, when drone is diving to follow a missing beacon.  
	// We could also just assign the depth setpoint much lower than current depth (like 10m), which will make the drone go basically full throttle down whenever it loses sight ... not sure what will work better
	PID->FollowingVertical=false;
	PID->FollowingHorizontal=false;
}

void *TopControlLoop(void * feedbackArgs){
	//convert from pointer to struct
	struct TopFeedbackStateStruct *TopFeedbackState = (struct TopFeedbackStateStruct *)feedbackArgs; 
	
	float roll_err_p, roll_err_i=0, roll_err_d, roll_err_prev=0;	
	float pitch_err_p, pitch_err_i=0, pitch_err_d, pitch_err_prev=0;
	float yaw_err_p, yaw_err_i=0, yaw_err_d, yaw_err_prev=0;
	
	float vx_err_p, vx_err_i=0, vx_err_d, vx_err_prev=0;
	float z_err_p, z_err_i=0, z_err_d, z_err_prev=0;
	float roll_correction, pitch_correction, yaw_correction, vx_correction, z_correction;
	struct timespec tLast,tNow;
	initTopPID(TopFeedbackState);
	InitializeMotors();
	delay(1000);
	struct AxisFeedbackState *RollAxisState 	= TopFeedbackState->rollState;
	struct AxisFeedbackState *PitchAxisState 	= TopFeedbackState->pitchState;
	struct AxisFeedbackState *YawAxisState 		= TopFeedbackState->yawState;
	struct AxisFeedbackState *VxAxisState		= TopFeedbackState->VxState;
	struct AxisFeedbackState *ZAxisState 		= TopFeedbackState->zState;
	clock_gettime(CLOCK_MONOTONIC_RAW, &tLast);
	uint64_t tDelta;
	while (1){
		clock_gettime(CLOCK_MONOTONIC_RAW, &tNow);
		tDelta = (uint64_t)((tNow.tv_sec - tLast.tv_sec)*1000000 + (tNow.tv_nsec - tLast.tv_nsec)/1000); // in microseconds
		if (tDelta > PID_SAMPLE_TIME) {
			// get current errors
			// ----------------------------		
			// roll
			//roll_err_p = RollAxisState->KP*(RollAxisState->currentValue - RollAxisState->setpoint);
			roll_err_p = RollAxisState->KP*(gArgs->x - RollAxisState->setpoint);
			roll_err_i += RollAxisState->KI*roll_err_p;
			roll_err_d = RollAxisState->KD*(roll_err_prev - roll_err_p);
			roll_err_prev = roll_err_p;
			roll_correction = roll_err_p+roll_err_i+roll_err_d;
			
			// pitch
			pitch_err_p = PitchAxisState->KP*(gArgs->y - PitchAxisState->setpoint);
			pitch_err_i += PitchAxisState->KI*pitch_err_p;
			pitch_err_d = PitchAxisState->KD*(pitch_err_prev - pitch_err_p);
			pitch_err_prev = pitch_err_p;
			pitch_correction = pitch_err_p+pitch_err_i+pitch_err_d;
			

			// yaw
			yaw_err_p = YawAxisState->KP*(*(YawAxisState->currentValue) - YawAxisState->setpoint);
			yaw_err_i += YawAxisState->KI*yaw_err_p;
			yaw_err_d = YawAxisState->KD*(yaw_err_prev - yaw_err_p);
			yaw_err_prev = yaw_err_p;
			yaw_correction = yaw_err_p+yaw_err_i+yaw_err_d;

			// x-velocity
			vx_err_p = VxAxisState->KP*(*(VxAxisState->currentValue) - VxAxisState->setpoint);
			vx_err_i += VxAxisState->KI*vx_err_p;
			vx_err_d = VxAxisState->KD*(vx_err_prev - vx_err_p);
			vx_err_prev = vx_err_p;
			vx_correction = vx_err_p+vx_err_i+vx_err_d;
			


			// z	
			z_err_p = ZAxisState->KP*(*(ZAxisState->currentValue) - ZAxisState->setpoint);
			z_err_i += ZAxisState->KI*z_err_p;
			z_err_d = ZAxisState->KD*(z_err_prev - z_err_p);
			z_err_prev = z_err_p;
			z_correction = z_err_p+z_err_i+z_err_d;
			//printf("\nZ Correction: %0.4f\n",z_correction);	
			// -------------------------
			// Modify motor thrust settings according to PID outputs
			// 
			//
			//
			//mArgs->leftTopMotorPower += roll_correction/2;
			//mArgs->rightTopMotorPower -= roll_correction/2;
			updateMotorStates(roll_correction, pitch_correction, yaw_correction, vx_correction, z_correction, TopFeedbackState->FollowingVertical);
			// --ADD THE REST HERE--
			//
			clock_gettime(CLOCK_MONOTONIC_RAW, &tLast); // reset timer

		}

	}
	pthread_exit(NULL);
}
