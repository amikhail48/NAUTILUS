/* UltrasonicEventChecker.c
 * Underwater Drone/ROV
 * Checks for changes in readings of the sensors
 */

#include <stdio.h>
#include <stdlib.h>
#include "EventFramework.h"
#include "HSM.h"
#include "eventQueue.h"
#include "SubUltrasonicEvent.h"
#include "autoMotorFunctions.h"
#include "UltrasonicEventChecker.h"

#define DEBOUNCE_TIMER 5

static uint8_t lastState = 0;
static uint8_t pendingState = 0;
static uint8_t debounceCounter;

Event checkUltrasonicSensors(){
	Event thisEvent = {No_Event, 0};
	uint8_t currentState = 0;

	uint8_t left = !digitalRead(leftPin);
	uint8_t right = !digitalRead(rightPin);
	uint8_t top = !digitalRead(upPin);
	uint8_t bottom = !digitalRead(downPin);

	// Move this to a new file : tracking = digitalRead(beaconPin);
	currentState = (left<<3) | (right<<2) | (top<<1) | bottom; //combine 4 digit reading
	printf("Current Ultrasonic State: %d %d %d %d\n", left, right, top, bottom);
	//printf("Pending State: %d\n", pendingState);

	if (debounceCounter == 0){
		//Start pending state
		//printf("Enter Pending\n");
		if (currentState != lastState){
			pendingState = currentState;
			debounceCounter++;
		}
	} else if (debounceCounter < DEBOUNCE_TIMER){
		//printf("debounceCounter: %d\n", debounceCounter);
		//Waiting in pending state
		if (currentState != pendingState){
			debounceCounter = 0;
		} else {
			debounceCounter++;
		}
	} else {
		//printf("Event Ready\n");
		//Pending state is ready
		lastState = pendingState;
		debounceCounter = 0;
		thisEvent.Type = Ultrasonic_Event;
		thisEvent.Param = currentState;
	}
	
	return thisEvent;
}
