#ifndef FEEDBACK_MOTOR_CONTROLLER_H
#define FEEDBACK_MOTOR_CONTROLLER_H

//#include "receiver.h"
#include <stdint.h>
struct MotorThrottleStateStruct {
	
	double frontTopMotorThrottle;
	double leftTopMotorThrottle;
	double rightTopMotorThrottle;
	double leftThrustMotorThrottle;
	double rightThrustMotorThrottle;
};


void InitializeMotors();
void updateMotorStates(float roll_correction, float pitch_correction, float yaw_correction, float vx_correction, float z_correction, uint8_t isFollowingVertical);

#endif
