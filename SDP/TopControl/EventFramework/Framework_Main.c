#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>
#include <pthread.h>
#include "EventFramework.h"
#include "UltrasonicEventChecker.h"
#include "DepthEventChecker.h"
#include "HumidityEventChecker.h"
#include "ADCEventChecker.h"
#include "eventQueue.h"
#include "HSM.h"
#include <wiringPi.h>
#include <time.h>
#include "../MS5803_Collection.h"
#include "MCP3202.h"

pthread_t tid[1];
struct timespec now;

const char* EventStr[] = {
	"No_Event",
	"Init_Event",
	"Entry_Event",
	"Exit_Event",
	"Ultrasonic_Event",
	"Distance_Event",
	"Depth_Event",
	"Humidity_Event"
};

//Private function declarations
void printEventOptions(void);


#ifdef TEST_EVENT_FRAMEWORK

int main(int argc, char* argv[]) {

	//Initialize everything..
	Queue hsmQueue = InitHSM();
	wiringPiSetup();
	gpioInitialise();
	initMotors();
	time_t timeStart = time(NULL);
	const int SPIChan = 0;
	const int SPIFreq = 2000000;
	MCP3202Setup(ADC_PINBASE,SPIChan,SPIFreq);
	
	//mArgs = malloc(sizeof(struct motorControl_args));
	psArgs = malloc(sizeof(struct presCollection_args));
	//uint8_t *tpArgs;
	// Start individual threads
	uint8_t err;
	/*err = pthread_create(&(tid[1]), NULL, &receiveCmds, (void*)mArgs);
	if (err != 0) {
		perror("Motor thread error");
	} else {
		printf("Motor Control thread created\n");
	} */
	/*err = pthread_create(&(tid[0]), NULL, &runPresCollection, (void*)psArgs);
	if (err != 0) {
		perror("Pressure thread error");
	} else {
		printf("Pressure thread created\n");
	}*/
	time_t timeNow;
	time_t lastTime = 0;
	//Start main loop
	int loop = TRUE;
	while (loop){
		delay(200);
        	timeNow = time(NULL);
		int timeRunning = timeNow-timeStart;
		//clock_gettime(CLOCK_REALTIME, &now);
		printf("time: %u\n", timeRunning);
#ifndef USE_KEYBOARD_INPUT
	//Put in wiringPi interrupt
		//Check for new events
		Event event = checkUltrasonicSensors();
		if (event.Type != No_Event){
			PostHSM(event);
	
		}
		printf("Ultrasonic result: %s\n", EventStr[event.Type]);

		/*Event event2 = checkDepthSensor();
		if(event2.Type != No_Event){
			PostHSM(event2);
		}
		printf("Depth result: %s\n", EventStr[event2.Type]);

		if((timeNow - lastTime) > 2){
			Event event3 = checkHumiditySensor();
			if(event3.Type != No_Event){
				PostHSM(event3);
			}
			printf("Humidity result: %s\n\n", EventStr[event3.Type]);
			lastTime = timeNow;
		}

		Event event4 = checkDroneDistance();
		if(event4.Type != No_Event){
			PostHSM(event4);
		}
		printf("Drone Tracking result: %s\n", EventStr[event4.Type]);*/

#endif


#ifdef USE_KEYBOARD_INPUT
		//Use user input to simulate state machine
		char in[] = {0};
		Event e1 = {No_Event, 0};

		printEventOptions();
		while ((in[0] - '0') > 7 || (in[0] - '0') < 1){
			printf(ANSI_COLOR_GREEN"Enter an Event Type (1 - 7): ");
			scanf("%s", in);
		}
		e1.Type = (EventType) (in[0] - '0' - 1);

		do {
			printf("Enter a Parameter (0 - 9): "ANSI_COLOR_RESET);
			scanf("%s", in);
		} while ((in[0] - '0') > 9 || (in[0] - '0') < 0);

		e1.Param = (uint8_t) in[0] - '0';
		printf("passing: %s\n", EventStr[e1.Type]);
		PostHSM(e1);
#endif //USE_KEYBOARD_INPUT 

		while (queueSize(hsmQueue) > 0){
			//Remove event from queue, and pass it to the HSM
			Event retEvent = RunHSM(removeEvent(hsmQueue));

			if (retEvent.Type != No_Event){
				printf(ANSI_COLOR_RED"Error: Event was not handled in HSM - %s" ANSI_COLOR_RESET "\n", EventStr[retEvent.Type]);
				loop = FALSE;
			}
		}
	}

	// TODO: Need a way to exit while loop
	freeQueue(&hsmQueue);
        free(psArgs);
	pthread_exit(NULL);
	return 0;
}

#else

void *runFramework(void *StateMachineArgs){	
	
	const int SPIChan = 0;
	const int SPIFreq = 2000000;
	
	//Initialize everything..
	Queue hsmQueue = InitHSM();
	StateMachineArgs = (struct StateMachineArgsObj *)StatemachineArgs;

	//wiringPiSetup();
	//gpioInitialise();
	//initMotors();
	MCP3202Setup(ADC_PINBASE,SPIChan,SPIFreq);
	
	time_t timeStart;
	uint8_t err;
	time_t timeNow;
	time_t lastTime = 0;
	uint32_t timeRunning;
	int loop = TRUE;
        clock_gettime(CLOCK_MONOTONIC_RAW, &timeStart);
	//Start main loop
	while (loop){
		delay(50);
		clock_gettime(CLOCK_MONOTONIC_RAW, &timeNow);
		timeRunning = (uint32_t)(timeNow.tv_usec-timeStart.tv_usec)/1000;
		//printf("time: %lu\n", timeRunning);
#ifndef USE_KEYBOARD_INPUT
	//Put in wiringPi interrupt
		//Check for new events
		Event event = checkUltrasonicSensors();
		if (event.Type != No_Event){
			PostHSM(event);
			//loop = FALSE;
		}
		//printf("Ultrasonic result: %s\n", EventStr[event.Type]);

		Event event2 = checkDepthSensor();
		if(event2.Type != No_Event){
			PostHSM(event2);
		}
		//printf("Depth result: %s\n", EventStr[event2.Type]);

		if((timeNow - lastTime) >= 2000){
			Event event3 = checkHumiditySensor();
			if(event3.Type != No_Event){
				PostHSM(event3);
			}
		//	printf("Humidity result: %s\n", EventStr[event3.Type]);
			lastTime = timeNow;
		}

		/*Event event4 = checkDroneDistance();
		if(event4.Type != No_Event){
			PostHSM(event4);
		}
		//printf("Drone Tracking result: %s\n", EventStr[event4.Type]);*/

#endif


#ifdef USE_KEYBOARD_INPUT
		//Use user input to simulate state machine
		char in[] = {0};
		Event e1 = {No_Event, 0};

		printEventOptions();
		while ((in[0] - '0') > 7 || (in[0] - '0') < 1){
			printf(ANSI_COLOR_GREEN"Enter an Event Type (1 - 7): ");
			scanf("%s", in);
		}
		e1.Type = (EventType) (in[0] - '0' - 1);

		do {
			printf("Enter a Parameter (0 - 9): "ANSI_COLOR_RESET);
			scanf("%s", in);
		} while ((in[0] - '0') > 9 || (in[0] - '0') < 0);

		e1.Param = (uint8_t) in[0] - '0';
		printf("passing: %s\n", EventStr[e1.Type]);
		PostHSM(e1);
#endif //USE_KEYBOARD_INPUT 

		while (queueSize(hsmQueue) > 0){
			//Remove event from queue, and pass it to the HSM
			Event retEvent = RunHSM(removeEvent(hsmQueue));

			if (retEvent.Type != No_Event){
				printf(ANSI_COLOR_RED"Error: Event was not handled in HSM - %s" ANSI_COLOR_RESET "\n", EventStr[retEvent.Type]);
				loop = FALSE;
			}
		}
	}

	// TODO: Need a way to exit while loop
	freeQueue(&hsmQueue);
        free(psArgs);
	pthread_exit(NULL);
	return 0;
}

#endif


//Private function definitions

void printEventOptions(void){
	printf(ANSI_COLOR_BLUE"\n1) No_Event    2) Init_Event    3) Entry_Event    4) Exit_Event\n");
	printf("5) Ultrasonic_Event    6) Depth_Event    7) Humidity_Event\n\n" ANSI_COLOR_RESET);
}

