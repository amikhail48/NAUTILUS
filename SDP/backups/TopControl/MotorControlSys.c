// MotorControlSys.c
// ----------------------


#include "autoMotorFunctions.h"
#include <pthread.h>
#include <stdlib.h>


struct TopFeedbackState {
	struct AxisFeedbackState rollState;
	struct AxisFeedbackState pitchState;
	struct AxisFeedbackState yawState;
	struct AxisFeedbackState zState;
};
struct AxisFeedbackState {
	float setpoint;
	float currentValue;
	float KP;
	float KI;
	float KD;
};

void *TopControlLoop(void * feedbackArgs){
	//convert from pointer to struct
	struct TopFeedbackState = *(feedbackArgs); 
	struct AxisFeedbackState RollAxisState = TopFeedbackState.rollState;
	float roll_err_p, roll_err_i, roll_err_d, roll_err_prev;
	roll_err_i=0;
	float roll_correction, pitch_correction, yaw_correction, depth_correction;
	while (1){
		// get current errors
		// ----------------------------		
		// roll
		roll_err_p = RollAxisState.KP*(RollAxisState.currentValue - RollAxisState.setpoint);
		roll_err_i += RollAxisState.KI*roll_err_p;
		roll_err_d = RollAxisState.KD*(roll_err_prev - roll_err_p);
		roll_correction = roll_err_p+roll_err_i+roll_err_d;
		// pitch
		// ADD PITCH PID HERE
		//




		// yaw
		// ADD YAW PID HERE
		//
		//


		// z
		// ADD Z PID HERE
		//
		//


		// -------------------------
		// Modify motor thrust settings according to PID outputs
		// roll
		//
		//

		mArgs->roll_correction=roll_correction;
		// --ADD THE REST HERE--
		//


		delay(100); // 10 samples/sec



	}
	pthread_exit(NULL);
}
