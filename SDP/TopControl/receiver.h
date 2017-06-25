#ifndef RECEIVER_H
#define RECEIVER_H
#include <stdint.h>
#include "FeedbackMotorController.h"
//BCM Pin Numbers
#define TEST_MOTOR_KEYINPUT
//struct motorControl_args *mArgs;
void *receiveCmds(void *mArgs_priv);

#endif
