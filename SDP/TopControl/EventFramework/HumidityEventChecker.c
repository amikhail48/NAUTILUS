/* Event Checker for Humidity Sensor inside Drone
 * Underwarer ROV/Drone
 */

#include <stdlib.h>
#include <stdio.h>
#include "EventFramework.h"
#include "HSM.h"
#include "eventQueue.h"
#include "humiditySensor.h"

int* dht11_data;
float f;
static uint8_t lastEvent = HUMIDITY_OK;
Event checkHumiditySensor(){
	Event thisEvent = {No_Event, 0};
	dht11_data = read_dht11_dat();
	f = dht11_data[2] * 9. / 5. + 32;
	printf( "Humidity = %d.%d %% Temperature (Inside Drone) = %d.%d C (%.1f F)\n", 
			dht11_data[0], dht11_data[1], dht11_data[2], dht11_data[3], f );
        if((dht11_data[0] > 80) && (lastEvent != HUMIDITY_HIGH)){
		thisEvent.Type = Humidity_Event;
		thisEvent.Param = HUMIDITY_HIGH;
		lastEvent = HUMIDITY_HIGH;
	}
	else if((dht11_data[0] < 80) && (lastEvent!=HUMIDITY_OK)){
		thisEvent.Type = Humidity_Event;
		thisEvent.Param = HUMIDITY_OK;
		lastEvent = HUMIDITY_OK;
	}
	return thisEvent;
}
