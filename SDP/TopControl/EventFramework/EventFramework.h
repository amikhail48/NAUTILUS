/* EventFramework.h
Contains declarations of structs and data-types used in the EventFramework */

#ifndef EVENT_FRAMEWORK_H

#define EVENT_FRAMEWORK_H

//Uncomment this to simulate the Framework from keyboard
//#define USE_KEYBOARD_INPUT 
#define TEST_EVENT_FRAMEWORK
#include <stdint.h>

#define ANSI_COLOR_RED		"\x1b[31m"
#define ANSI_COLOR_GREEN	"\x1b[32m"
#define ANSI_COLOR_BLUE		"\x1b[34m"
#define ANSI_COLOR_RESET	"\x1b[0m"

#define TOO_DEEP	0
#define TOO_SHALLOW	1
#define DEPTH_OK        2
#define HUMIDITY_HIGH   3
#define HUMIDITY_OK	4
#define TRACKING	5
#define LOST_SIGNAL	6

typedef enum {
	No_Event,
	Init_Event,
	Entry_Event,
	Exit_Event,
	Ultrasonic_Event,
	Distance_Event,
	Depth_Event,
	Humidity_Event
} EventType;


typedef enum {
    FALSE,
    TRUE
} bools;

extern const char* EventStr[]; //Defined in Framework_Main.c

typedef struct Event{
	EventType Type;
	uint8_t Param;
} Event;

#endif //EVENT_FRAMEWORK_H
