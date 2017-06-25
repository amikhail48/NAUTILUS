import os
import sys

path = "/home/pi/SDP/python_pipe/fifo"


line=""
running=True
def read_fifo(filename):
	while (True):
		with open(filename) as fifo:
			yield fifo.read()
def main():
	if (os.path.exists(path)== False):
		print path,"does not exist, creating named pipe"
		os.mkfifo(path)
	for line in read_fifo(path):
		print "Received: "+line
		if (line == "end"):
			sys.exit()
						
main()

