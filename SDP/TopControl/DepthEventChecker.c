/* Event Checker for Depth Sensor
 * Underwater ROV/Drone
 */

#include <stdlib.h>
#include <stdio.h>
#include "EventFramework.h"
#include "HSM.h"
#include "eventQueue.h"
#include "MS5803_Collection.h"

float pressure = 0;
static uint8_t lastEvent = DEPTH_OK;
Event checkDepthSensor(){
    Event thisEvent = {No_Event, 0};
    pressure = psArgs->pres;
    printf("Pressure is: %0.2f\n", pressure);
    if(pressure<1.001 && lastEvent!=TOO_SHALLOW){
         thisEvent.Type = Depth_Event;
	 thisEvent.Param = TOO_SHALLOW;
	 lastEvent = TOO_SHALLOW;
    }
    else if(pressure>10 && lastEvent!=TOO_DEEP){
         thisEvent.Type = Depth_Event;
	 thisEvent.Param = TOO_DEEP;
	 lastEvent = TOO_DEEP;
    }
    else if( 10 < pressure && pressure < 1.001 && lastEvent!=DEPTH_OK){
         thisEvent.Type = Depth_Event;
	 thisEvent.Param = DEPTH_OK;
	 lastEvent = DEPTH_OK;
    }
   //printf("%d\n", thisEvent.Param);
    return thisEvent;
}
