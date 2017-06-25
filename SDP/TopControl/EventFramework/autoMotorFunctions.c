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
#include "autoMotorFunctions.h"

#define MAX_LINE 6

#define NEUTRAL_THROTTLE	1500
#define PIN_LTHRUST		19
#define PIN_RTHRUST		16
#define PIN_LTOP		26
#define PIN_RTOP		20
#define PIN_FTOP		21
#define NUM_MOTORS		5

#define beaconPin		25
#define rightPin		1
#define leftPin			4
#define upPin			5
#define downPin			6
#define LEDPin			26

const char pinList[5] = {PIN_LTHRUST, PIN_RTHRUST, PIN_LTOP, PIN_RTOP, PIN_FTOP};

//Motor Functions

void goForward(int percent) { //Use negative to reverse
	int pulse;
	pulse = NEUTRAL_THROTTLE+(500*percent)/100;
	gpioServo(PIN_LTHRUST, pulse);
	gpioServo(PIN_RTHRUST, pulse);
}

void turnLeft(int percent) {
	int pulse;
	pulse = NEUTRAL_THROTTLE+(500*percent)/100;
	gpioServo(PIN_RTHRUST, pulse);
	gpioServo(PIN_LTHRUST, NEUTRAL_THROTTLE);
}

void turnRight(int percent) {
	int pulse;
	pulse = NEUTRAL_THROTTLE+(500*percent)/100;
	gpioServo(PIN_LTHRUST, pulse);
	gpioServo(PIN_RTHRUST, NEUTRAL_THROTTLE);
}

void dive(int percent) { //Use negative percent to rise
	int pulse;
	pulse = NEUTRAL_THROTTLE+(500*percent)/100;
	gpioServo(PIN_LTOP, pulse);
	gpioServo(PIN_RTOP, pulse);
	gpioServo(PIN_FTOP, pulse);
}

void initMotors() {
	uint8_t i;
	for (i=0;i<NUM_MOTORS;i++){
		gpioServo(pinList[i], NEUTRAL_THROTTLE);
	}

}
/*int right, left, up, down, tracking;
int main{
	right = digitalRead(rightPin);
	left = digitalRead(leftPin);
	up = digitalRead(upPin);
	down = digitalRead(downPin);
	tracking = digitalRead(beaconPin);
	if(right) turnRight(10);
	if(!right) turnRight(0);
	if(left) turnLeft(10);
	if(!left) turnLeft(0);
	if(up) dive(-10);
	if(!up) dive(0);
	if(down) dive(10);
	if(!down) dive(0);
	if(!tracking) digitalWrite(LEDPin, HIGH);
	return 0;
}
delay(150);*/
//pthread_exit(NULL);
