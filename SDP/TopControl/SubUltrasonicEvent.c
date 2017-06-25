/* SubUltrasonicEvent.c
 * Substate Machine for Ultrasonic Events
 * Underwater ROV/Drone
 */

#include <stdio.h>
#include <stdlib.h>
#include "EventFramework.h"
#include "HSM.h"
#include "eventQueue.h"
#include "SubUltrasonicEvent.h"
#include "FeedbackControlTop.h"

typedef enum {
    Init,
    Tracking,
} SubUltrasonicEventHSMState;

static SubUltrasonicEventHSMState CurrentState;
int right, left, up, down, tracking, moving;
struct AxisFeedbackState *YawSetpoint; 	    
struct AxisFeedbackState *PitchSetpoint;
struct AxisFeedbackState *VxSetpoint;	
struct TopFeedbackStateStruct *TopSetpoint; 
extern struct TopFeedbackStateStruct *fbArgs;
extern struct MotorThrottleStateStruct *MotorThrottleState;
int pivotToSignal(uint8_t param);

uint8_t InitSubUltrasonicState(){
    TopSetpoint = fbArgs;//(struct TopFeedbackStateStruct *)mArgs;
    YawSetpoint 	= TopSetpoint->yawState;    
    PitchSetpoint 	= TopSetpoint->pitchState;
    VxSetpoint		= TopSetpoint->VxState;
    Event returnEvent;
    CurrentState = Init;
    Event initEvent = {Init_Event, 0};
    returnEvent = RunSubUltrasonicEventHSM(initEvent);
    if (returnEvent.Type == No_Event) {
        return TRUE;
    }
    return FALSE;
}

Event RunSubUltrasonicEventHSM(Event ThisEvent) {
    uint8_t changeStates = FALSE; // use to flag transition
    SubUltrasonicEventHSMState NextState;
    
    switch (CurrentState) {
        case Init:
            if(ThisEvent.Type == Init_Event){
                //do initializations
                
                NextState = Tracking;
                changeStates = TRUE;
            }
            break;

        case Tracking:
            if (ThisEvent.Type == Ultrasonic_Event){
                if (ThisEvent.Param == 0){
                    //Lost the signal
                    return ThisEvent;
                }
		YawSetpoint->setpoint = 0;
		PitchSetpoint->setpoint = 0;
		/*turnLeft(0);
		turnRight(0);
		dive(0);
		goForward(0);*/

                pivotToSignal(ThisEvent.Param);
                ThisEvent.Type = No_Event;
            }

        

        default://NOT SURE WHAT THIS IS?
            break;

    }
    if (changeStates == TRUE) { // making a state transition, send EXIT and ENTRY
 		// recursively call the current state with an exit event
        Event exitEvent = {Exit_Event, 0};
        Event entryEvent = {Entry_Event, 0};
        RunSubUltrasonicEventHSM(exitEvent);
        CurrentState = NextState;
        RunSubUltrasonicEventHSM(entryEvent);
    }
    return ThisEvent;
}


/*Adjusts the direction the drone is facing/moving depending on the 
state of the ultrasonic sensors. Param is in the following order: LRTB
Returns 1 for ok, and 0 for a problem */
int pivotToSignal(uint8_t param){
    //Move drone depending on ultrasonic parameter
    //int moving = 0;
    switch (param){
        
        case TOP_ON:
        case (TOP_ON | LEFT_ON | RIGHT_ON): //0b1110
            //Go up
	    //
            //dive(10);

// See my comment initTopPID (in FeedbackControlTop.c)
// we could also do this instead:
// 	    TopSetpoint->zAxisState->setpoint -= 100 (cm)
// then set it to current value when it finds the beacon
	    TopSetpoint->FollowingVertical=TRUE;
	    MotorThrottleState->leftTopMotorThrottle-=20; //msec pulse width, not percent
	    MotorThrottleState->rightTopMotorThrottle-=20;
	    PitchSetpoint->setpoint += (10);  
            break;

        case BOTTOM_ON:
        case (BOTTOM_ON | LEFT_ON | RIGHT_ON): //0b1101
            //Go down
            //dive(10);
	    TopSetpoint->FollowingVertical=TRUE;
	    MotorThrottleState->leftTopMotorThrottle+=35; //msec pulse width, not percent
	    MotorThrottleState->rightTopMotorThrottle+=35;
	    
	    PitchSetpoint->setpoint -= (10);
            break;

        case LEFT_ON:
        case (LEFT_ON | TOP_ON | BOTTOM_ON): //0b1011
            //Go left
	    //turnLeft(10);
	    TopSetpoint->FollowingHorizontal=TRUE;
	    YawSetpoint->setpoint -= (10);
            break;

        case RIGHT_ON:
        case (RIGHT_ON | TOP_ON | BOTTOM_ON): //0b0111
            //Go right
	    //turnRight(10);
	    YawSetpoint->setpoint += (10);
            break;

        case (TOP_ON | LEFT_ON): //0b1010
            //Go up and to the left
	   // dive(10);
	   // turnLeft(10);
	   PitchSetpoint->setpoint += (10);
	   YawSetpoint->setpoint -= (10);
           break;

        case (TOP_ON | RIGHT_ON): //0b0110
	    printf("left OFF!\n");
            //Go up and to the right
	    //dive(10);
	    //turnRight(10);
	    PitchSetpoint->setpoint += (10);
	    YawSetpoint->setpoint += (10);
            break;

        case (BOTTOM_ON | LEFT_ON): //0b1001
            //Go down and to the left
	    //dive(10);
	    //turnLeft(10);
	    PitchSetpoint->setpoint -= (10);
	    YawSetpoint->setpoint -= (10);
            break;

        case (BOTTOM_ON | RIGHT_ON): //0b0101
            //Go down and to the right
	    //dive(10);
	    //turnRight(10);
	    PitchSetpoint->setpoint -= (10);
	    YawSetpoint->setpoint += (10);
            break;

        default:
//8==========================D
	    // YO - Does this get called repeatedly all the beacons stay on or only once when it finds all of them again?
//8==========================D
	    // ALL_ON || LEFT_ON | RIGHT_ON || TOP_ON | BOTTOM_ON
            //Go forward
	    //goForward(3);
	    //TopSetpoint->FollowingVertical=FALSE;
	    //TopSetpoint->FollowingHorizontal=FALSE;
	    // Set the new target velocity based on current value because it tends to drift from 0
	    // We can test this, but we might have to do forward velocity manually b/c it gets unreliable as time goes on
	   // VxSetpoint->setpoint = *(VxSetpoint->currentValue)+10; 
            break;
    }

    return 1;
}
