/* HSM.h
 * Hierarchal State Machine
 * Underwater ROV/Drone
 */

#ifndef HSM_H
#define HSM_H

#include "EventFramework.h"
#include "eventQueue.h"


/* Sets up hsmQueue, initializes states
Returns 0 for Fail and 1 for Success */
Queue InitHSM(void);

/* Posts a new event to the hsmQueue
Returns 0 for Fail and 1 for Success */
uint8_t PostHSM(Event ThisEvent);

/* Runs the HSM on the next event in its queue
Will return No_Event if the event is handled,
otherwise the event was not handled and program should exit */
Event RunHSM(Event ThisEvent);


#endif //HSM_H