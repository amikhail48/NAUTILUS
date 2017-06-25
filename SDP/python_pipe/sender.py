import os
import sys
import time
path = "/home/pi/SDP/python_pipe/fifo"

def main():
	if (len(sys.argv)==1):
		return 0;
	#fifo = open(path, "a")
	for i in sys.argv[1:len(sys.argv)]:
		fifo = open(path, "a")
		fifo.write(i)
		fifo.write("\n")
		fifo.close()
		#time.sleep(0.1)
	#fifo.close()
main()
