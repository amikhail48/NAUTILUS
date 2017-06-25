#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#define MAX_LINE 80

int main(int argc, char** argv) {
   char line[MAX_LINE];
   int pipe;
   if( mkfifo( "myFIFO", 0666 ) < 0 )
    {
        printf( "Cannot create a pipe\n" );
        //exit( EXIT_FAILURE );
    }
   // open a named pipe
   printf("Enter line: ");
   fgets(line, MAX_LINE, stdin);
   pipe = open("/home/pi/SDP/gpipe/myFIFO", O_WRONLY);

   // get a line to send

   // actually write out the data and close the pipe
   write(pipe, line, strlen(line));

	// close the pipe
   close(pipe);
   return 0;
}
