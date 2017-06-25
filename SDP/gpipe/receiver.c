#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <wiringPi.h>
#include <string.h>

#define MAX_LINE 80 
#define WRITE_CMD "wr"

int main(int argc, char** argv) {
   char wr_cmd[2];
   strcpy(wr_cmd, "wr");
   char line[MAX_LINE];
   int pipe;
   int bytes_read;
   wiringPiSetup();
   // open a named pipe
   pipe = open("/home/pi/SDP/gpipe/myFIFO", O_RDONLY);
   while (1) {
      if ((bytes_read = read(pipe, line, MAX_LINE)) > 0){
         line[bytes_read]='\0';
	 //printf("%c", line);
	// if (strcmp(line,wr_cmd) == 0){
	//	printf("Writing\n");
	//	digitalWrite((int)(line[2]), (int)(line[3]));
	//}
         printf("Received: %s\n",line);
      }
	time_sleep(0.1);
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
   return 0;
}
