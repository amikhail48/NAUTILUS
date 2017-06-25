CC=gcc
CFLAGS=-Wall
LINKS=-lwiringPi -lwiringPiDev -lpigpio -lpthread -lm
SOURCE=Framework_Main.c eventQueue.c HSM.c SubUltrasonicEvent.c ADCEventChecker.c HumidityEventChecker.c DepthEventChecker.c UltrasonicEventChecker.c autoMotorFunctions.c humiditySensor.c ../MS5803_Collection.c ../ms5803.c MCP3202.c
EXECUTABLE=FrameworkMain
 
all : $(EXECUTABLE)

$(EXECUTABLE) : $(SOURCE)
	$(CC) $(CFLAGS) -o $(EXECUTABLE) $(SOURCE) $(LINKS)

valgrind : $(EXECUTABLE)
	valgrind -v --leak-check=yes ./$(EXECUTABLE)

clean :
	rm -f $(EXECUTABLE) $(OBJECTS)
