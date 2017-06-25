/* ADCEventChecker.h
 * Underwater Drone/ROV
 */

#ifndef ADCEVENT_CHECKER_H
#define ADCEVENT_CHECKER_H

#include <stdlib.h>
#include <stdio.h>
//#include <wiringPi.h>
#include "EventFramework.h"
#include "HSM.h"
#include "eventQueue.h"
#include <stdint.h>
#include "MCP3202.h"

Event checkDroneDistance(void);

#endif
