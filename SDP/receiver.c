#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <wiringPi.h>
#include <string.h>
#include <pigpio.h>

#define MAX_LINE 6 
//#define WRITE_CMD "FD"

#define NEUTRAL_THROTTLE	1500
#define PIN_LTHRUST		19
#define PIN_RTHRUST		16

void goForward(char percent) {
	int pulse;
	pulse = NEUTRAL_THROTTLE+(500*percent)/100;
	gpioServo(PIN_LTHRUST, pulse);
	gpioServo(PIN_RTHRUST, pulse);
}

int main(int argc, char** argv) {
   char wr_cmd[2];
   //printf("%s\n",wr_cmd);
   strcpy(wr_cmd, "FD");
   char keyup_cmd[2];
   char keydown_cmd[2];
   strcpy(keyup_cmd, "up");
   strcpy(keydown_cmd, "dn");
  
   char *line=malloc(sizeof(char)*6);
   char *sub_line;
   int pipe;
   int cmdVal;
   int bytes_read;
   char dir;
   if (gpioInitialise()<0) return -1;
   wiringPiSetup();
   // open a named pipe
   pipe = open("/var/www/html/myFIFO", O_RDONLY | O_NONBLOCK);
   while (1) {
      if ((bytes_read = read(pipe, line, MAX_LINE)) > 0){
         if (strncmp(line,keyup_cmd,2) == 0){
		dir=0;
		printf("Stop ");
	 } else if (strncmp(line,keydown_cmd,2)==0){
		dir=1;
		printf("Go ");
	 } 
	 sub_line = (line+2*sizeof(char));		
	 cmdVal = atoi(sub_line);
	 //line[bytes_read-1]='\0'; //replace LF char
	 //printf("%d\n",strcmp(line,wr_cmd));	 
	 if(cmdVal == 38) {//if (strcmp(line,wr_cmd) == 0){
		printf("Up\n");
	 } else if (cmdVal == 39){
		printf("Right\n");
	 } else if (cmdVal == 37) {
		printf("Left\n");
	 } else if (cmdVal == 40) {
		printf("Down\n");
	 } else if (cmdVal == 32) {
		if ( (line[0] == 'u') && (line[1] == 'p') ) {
			goForward(0);
		} else if ( (line[0] == 'd') && (line[1] == 'n') ) {
			goForward(10);
		}
		printf("Forward\n");	
	 } else
         	printf("Received: %s\n",line);
      }
      delay(50);
	// else
        // break;
   }	   
// get a line to send
//   printf("Enter line: ");
//   fgets(line, MAX_LINE, stdin);

   // actually write out the data and close the pipe
//   write(pipe, line, strlen(line));

	// close the pipe
   close(pipe);
   free(line);
   return 0;
}
