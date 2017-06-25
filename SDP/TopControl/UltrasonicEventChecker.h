/* UltrasonicEventChecker.h
 * Underwater Drone/ROV
 */

#ifndef ULTRASONIC_EVENT_CHECKER_H
#define ULTRASONIC_EVENT_CHECKER_H

#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>
#include <wiringPi.h>
#include "EventFramework.h"
#include "HSM.h"
#include "eventQueue.h"
#include "SubUltrasonicEvent.h"
#include "UltrasonicEventChecker.h"

#define beaconPin 25
#define rightPin 1
#define leftPin 4
#define upPin 5
#define downPin 21

Event checkUltrasonicSensors();

#endif //ULTRASONIC_EVENT_CHECKER_H
