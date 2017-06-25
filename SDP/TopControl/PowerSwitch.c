#include <stdlib.h>
#include <wiringPi.h>
#include <stdio.h>

#define PIN_POWERSWITCH	7 

void ISR_PowerSwitchDetect(void){
	printf("Shutting Down\n");
	delay(250);
	if (digitalRead(PIN_POWERSWITCH) == 0){
		system("sudo shutdown now");
	}
}
int main(){
	wiringPiSetup();
	pullUpDnControl(PIN_POWERSWITCH, PUD_UP);
	delay(100);
	wiringPiISR(PIN_POWERSWITCH, INT_EDGE_FALLING, &ISR_PowerSwitchDetect); 
	while (1){
		delay(1000);
	}
	return 0;
}
