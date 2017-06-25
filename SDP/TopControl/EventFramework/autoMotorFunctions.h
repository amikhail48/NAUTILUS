
#ifndef AUTO_MOTOR_FUNCS_H
#define AUTO_MOTOR_FUNCS_H

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <wiringPi.h>
#include <string.h>
#include <pigpio.h>
//#include "receiver.h"
#define MAX_LINE 6

#define NEUTRAL_THROTTLE	1500
#define PIN_LTHRUST		19
#define PIN_RTHRUST		16
#define PIN_LTOP		26
#define PIN_RTOP		20
#define PIN_FTOP		21
#define NUM_MOTORS		5

#define beaconPin		25
#define LEDPin			26

//const char pinList[5] = {PIN_LTHRUST, PIN_RTHRUST, PIN_LTOP, PIN_RTOP, PIN_FTOP};

//Motor Functions
void goForward(int percent);
void turnLeft(int percent);
void turnRight(int percent);
void dive(int percent);
void initMotors();

#endif //AUTO_MOTOR_FUNCS_H
