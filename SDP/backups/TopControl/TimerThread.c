#include <stdlib.h>
#include <pthread.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <stdint.h>
#define TIMER1_INT	1000 //nanoseconds
void runTimer(void *timerArgs){
	struct timespec start;
	clock_gettime(CLOCK_MONOTONIC,&start);
	struct timespec now;
	uint32_t deltaT;
	while(1){
		
		clock_gettime(CLOCK_MONOTONIC,&now);
		deltaT = (uint32_t)(now.tv_nsec-start.tv_nsec);
		if (deltaT > TIMER1_INT){
			printf("\r%lu INTERRUPT            ",deltaT);
			fflush(stdout);
			//reset start timer
			start = now;
		}
	}
}

int main(){
	runTimer(NULL);
	return 0;
}
