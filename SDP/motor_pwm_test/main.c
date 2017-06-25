#include <stdio.h>
#include <stdlib.h>

#include <pigpio.h>


#define MOTOR_A_PWM_PIN 19 //BCM notation
#define MOTOR_B_PWM_PIN 
#define MOTOR_C_PWM_PIN 

int main(){
	char input[4];
	int exit = 0;
	int speed;
	int pulse;
	printf("\tMOTOR TEST HARNESS\n=====================================\n");
	if (gpioInitialise()<0) return -1;
	time_sleep(1);
	printf("Drivers Initialized\n");
	gpioServo(MOTOR_A_PWM_PIN,1500);
//	gpioServo(MOTOR_B_PWM_PIN,1500);
	time_sleep(1);
	printf("PWM OUT on PIN %d\n",MOTOR_A_PWM_PIN);
	time_sleep(1);
	while(exit==0){
		printf("Enter speed (+/- 0-100): ");
		scanf("%s", input);
		if (input[0] != 'q'){
			if (input[0] == '-') {
				speed = (atoi(&input[1])*500)/100;
				pulse = 1500 - speed;
			} else {
				speed = (atoi(input)*500)/100;
				pulse = 1500 + speed;
			}
			if (pulse > 2000) {
				pulse = 2000;
			} else if (pulse < 1000) {
				pulse = 1000;
			}
			gpioServo(MOTOR_A_PWM_PIN, pulse);
//			gpioServo(MOTOR_B_PWM_PIN, pulse);
			printf("Sending %d us pulse\n",pulse);
		} else {
			exit = 1;
		}
	}
	printf("Stopping\n");
	gpioServo(MOTOR_A_PWM_PIN,1500);
//	gpioServo(MOTOR_B_PWM_PIN,1500);
	time_sleep(1);
	return 0;
}
