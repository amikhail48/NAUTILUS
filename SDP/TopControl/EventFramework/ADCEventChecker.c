/* ADCEventChecker.c
 * Underwater Drone/ROV
 */
//
//
#include <stdio.h>
#include <stdlib.h>
#include "ADCEventChecker.h"
#include "EventFramework.h"
#include <wiringPi.h>
#include "HSM.h"
#include "eventQueue.h"

#define DEBOUNCE_TIMER 10

static uint8_t lastEvent = TRACKING;
static uint8_t pendingState = 0;
static uint8_t debounceCounter = 0;

Event checkDroneDistance(void){
	float vref = 6.0;
	Event thisEvent = {No_Event, 0};
	unsigned int sample = analogRead(ADC1_PIN);
	float convertedVal = (float)sample*vref/4096;
	printf("\rSample: %d <----> %0.4f V\n",sample,convertedVal);
	fflush(stdout);

	uint8_t currentState = TRACKING;
	if (convertedVal < 4.0) {
		currentState = LOST_SIGNAL;
	}

	if (debounceCounter == 0){
		if (currentState != lastEvent){
			pendingState = currentState;
			debounceCounter++;
		}
	} else if (debounceCounter < DEBOUNCE_TIMER) {
		if (currentState != pendingState){
			debounceCounter = 0;
		} else {
			debounceCounter++;
		}
	} else {
		lastEvent = pendingState;
		debounceCounter = 0;
		thisEvent.Type = Distance_Event;
		thisEvent.Param = pendingState;
	}
	return thisEvent;
}
