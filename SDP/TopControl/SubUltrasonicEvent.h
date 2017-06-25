/* SubUltrasonicEvent.c
 * Substate Machine for Ultrasonic Events
 * Underwater ROV/Drone
 */

#ifndef SUB_ULTRASONIC_EVENT_H
#define SUB_ULTRASONIC_EVENT_H

#include <stdio.h>
#include <stdlib.h>
#include "EventFramework.h"
#include "HSM.h"

//Ultrasonic sensors oordered as: LRTB
#define LEFT_ON		0b1000
#define RIGHT_ON	0b0100
#define TOP_ON 		0b0010
#define BOTTOM_ON	0b0001

uint8_t InitSubUltrasonicState(void);
Event RunSubUltrasonicEventHSM(Event ThisEvent);

#endif //SUB_ULTRASONIC_EVENT_H
