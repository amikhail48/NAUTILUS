FLAGS	= -Wall -g 
LINKS	= -lpigpio -lwiringPi -lpthread -lm -lrt
SOURCES = Framework_Main.c eventQueue.c HSM.c SubUltrasonicEvent.c ADCEventChecker.c HumidityEventChecker.c DepthEventChecker.c UltrasonicEventChecker.c autoMotorFunctions.c humiditySensor.c MCP3202.c TopControl.c LSM6DS3_Collection.c receiver.c LinkedListStruct.c ms5803.c MS5803_Collection.c IMU.c FeedbackControlTop.c FeedbackMotorController.c
OBJECTS = Framework_Main.o eventQueue.o HSM.o SubUltrasonicEvent.o ADCEventChecker.o HumidityEventChecker.o DepthEventChecker.o UltrasonicEventChecker.o autoMotorFunctions.o humiditySensor.o MCP3202.o TopControl.o LSM6DS3_Collection.o receiver.o LinkedListStruct.o ms5803.o MS5803_Collection.o IMU.o FeedbackControlTop.o FeedbackMotorController.o
HEADERS = LSM6DS3_Collection.h LinkedListStruct.h receiver.h ms5803.h MS5803_Collection.h IMU.h FeedbackControlTop.h FeedbackMotorController.h
EXEBIN  = top

all:	$(EXEBIN)

$(EXEBIN) : $(OBJECTS) $(HEADERS)
	gcc -o $(EXEBIN) $(OBJECTS) $(LINKS)

$(OBJECTS) : $(SOURCES) $(HEADERS)
	gcc -c $(FLAGS) $(SOURCES)

clean : 
	rm -f $(EXEBIN) $(OBJECTS) 
