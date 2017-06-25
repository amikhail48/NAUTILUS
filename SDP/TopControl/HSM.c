/* HSM.c
 * Hierarchal State Machine
 * Underwater ROV/Drone
 */

#include <stdio.h>
#include <stdlib.h>

#include "EventFramework.h"
#include "HSM.h"
#include "eventQueue.h"
#include "SubUltrasonicEvent.h"

typedef enum {
    InitState,
    SearchingForSignal,
    TrackingSignal,
    GoingUp,
    GoingDown,
    Panic,
} HSMState;

static HSMState CurrentState = InitState;

static Queue hsmQueue;


/* Sets up hsmQueue, initializes states
Returns the pointer to the hsmQueue */
Queue InitHSM(void){
	hsmQueue = newQueue();
	CurrentState = InitState;
	Event initEvent = {Init_Event, 0};
	PostHSM(initEvent);
	return hsmQueue;
}

/* Posts a new event to the hsmQueue
Returns 0 for Fail and 1 for Success */
uint8_t PostHSM(Event ThisEvent){
	insertEvent(ThisEvent, hsmQueue);
	return 1;
}

/* Runs the HSM on the next event in its queue
Will return No_Event if the event is handled,
otherwise the event was not handled and program should exit */
Event RunHSM(Event ThisEvent){
	uint8_t changeStates = FALSE;
	HSMState NextState;
	Event newEvent; //Use to post to HSM

	switch(CurrentState){
		case InitState:
			printf("%s(%d) ->  InitState\n", EventStr[ThisEvent.Type], ThisEvent.Param);
			if(ThisEvent.Type == Init_Event){
				//do initializations

				//Init all sub-states
				InitSubUltrasonicState();
				
				NextState = SearchingForSignal;
				changeStates = TRUE;
			}
			ThisEvent.Type = No_Event;
			break;

		case SearchingForSignal:
			printf("%s(%d) ->  SearchingForSignal\n", EventStr[ThisEvent.Type], ThisEvent.Param);

			//Pass event to sub-state

			if(ThisEvent.Type == Ultrasonic_Event){
				newEvent.Type = ThisEvent.Type;
				newEvent.Param = ThisEvent.Param;
				PostHSM(newEvent); //To pass param to TrackingSignal

				NextState = TrackingSignal;
				changeStates = TRUE;
				ThisEvent.Type = No_Event;
	
			}
			else if(ThisEvent.Type == Depth_Event){
				//TODO: typedef params to somthing like TOO_DEEP & TOO_SHALLOW
				//DONE: I #defineed them in Event_Framework.h
				if (ThisEvent.Param == TOO_DEEP) {
					NextState = GoingUp;
					changeStates = TRUE;
				}
				else if (ThisEvent.Param == TOO_SHALLOW) {
					NextState = GoingDown;
					changeStates = TRUE;
				}
				ThisEvent.Type = No_Event;

			}
			else if(ThisEvent.Type == Humidity_Event){
				NextState = Panic;
				changeStates = TRUE;
				ThisEvent.Type = No_Event;

			}
			break;

		case TrackingSignal:
			printf("%s(%d) ->  TrackingSignal\n", EventStr[ThisEvent.Type], ThisEvent.Param);

			//Pass event to sub-state
			ThisEvent = RunSubUltrasonicEventHSM(ThisEvent);

			if(ThisEvent.Type == Depth_Event){
				//TODO: typedef params to somthing like TOO_DEEP & TOO_SHALLOW
				//DONE: I #defineed them in Event_Framework.h
				if (ThisEvent.Param == TOO_DEEP) {
					NextState = GoingUp;
					changeStates = TRUE;
				}
				else if (ThisEvent.Param == TOO_SHALLOW) {
					NextState = GoingDown;
					changeStates = TRUE;
				}
				ThisEvent.Type = No_Event;

			}
			else if(ThisEvent.Type == Humidity_Event){
				NextState = Panic;
				changeStates = TRUE;
				ThisEvent.Type = No_Event;

			}
			else if (ThisEvent.Type == Ultrasonic_Event) {
				//Lost the signal
				//LRTB
				if(ThisEvent.Param == 6){
					//turnLeft(10);
					printf("left off\n");
					//exit(1);
					//delay(30);
					//turnLeft(0);
				}
				/*else if(ThisEvent.Param == 1010){
					turnRight(10);
				}
				else if(ThisEvent.Param == 1100){
					dive(10);
				}
				else if(ThisEvent.Param == 1110){
					dive(-10);
				}*/
				NextState = SearchingForSignal;
				changeStates = TRUE;
				ThisEvent.Type = No_Event;
			}
			break;

		case GoingUp:
			printf("%s(%d) ->  GoingUp\n", EventStr[ThisEvent.Type], ThisEvent.Param);

			if(ThisEvent.Type == Ultrasonic_Event || ThisEvent.Type == Distance_Event){
				//ignore
				ThisEvent.Type = No_Event;	
			}
			else if(ThisEvent.Type == Depth_Event){
				if (ThisEvent.Param == 2) {
					NextState = SearchingForSignal;
					changeStates = TRUE;
				}
				ThisEvent.Type = No_Event;
			}
			else if(ThisEvent.Type == Humidity_Event){
				NextState = Panic;
				changeStates = TRUE;
				ThisEvent.Type = No_Event;
			}
			break;

		case GoingDown:
			printf("%s(%d) ->  GoingDown\n", EventStr[ThisEvent.Type], ThisEvent.Param);

			if(ThisEvent.Type == Ultrasonic_Event || ThisEvent.Type == Distance_Event){
				//ignore
				ThisEvent.Type = No_Event;	
			}
			else if(ThisEvent.Type == Depth_Event){
				if (ThisEvent.Param == 2) {
					NextState = SearchingForSignal;
					changeStates = TRUE;
				}
				ThisEvent.Type = No_Event;
			}
			else if(ThisEvent.Type == Humidity_Event){
				NextState = Panic;
				changeStates = TRUE;
				ThisEvent.Type = No_Event;
			}
			break;

		case Panic:
			printf("%s(%d) ->  Panic\n", EventStr[ThisEvent.Type], ThisEvent.Param);

			//Ignore all new events (for now)

			//Go to surface
			ThisEvent.Type = No_Event;
			break;

	} //End currentState switch

	if (changeStates == TRUE) {
		//Send exit event to current state, and entry to new state
		Event exit = {Exit_Event, 0};
		Event entry = {Entry_Event, 0};
		RunHSM(exit);
		CurrentState = NextState;
		RunHSM(entry);
	}

	return ThisEvent;
}
