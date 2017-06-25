#include <stdlib.h>
#include <stdio.h>
#include <pigpio.h>

#define NEUTRAL_THROTTLE	1500
#define PIN_LTHRUST		19
#define PIN_RTHRUST		16

void goForward(char percent) {
	int pulse;
	pulse = NEUTRAL_THROTTLE+(500*percent)/100;
	gpioServo(PIN_LTHRUST, pulse);
	gpioServo(PIN_RTHRUST, pulse);
	return NULL;
}

int main() {
	return 0;
}
