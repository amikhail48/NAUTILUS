#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>

void tapISR(void) {
	printf("INTERRUPT\n");
}
int main() {
        wiringPiSetup();
	if(wiringPiISR(16,INT_EDGE_RISING,&tapISR)==-1) printf("ERROR\n");
	while(1){
		//if (digitalRead(10)==1){
		//	printf("INT");
		//}
		;
	}
	return 0;
} 
