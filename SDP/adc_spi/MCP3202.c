#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <stdlib.h>
#include <stdio.h>
#include "MCP3202.h"
#include <errno.h>

static int myAnalogRead (struct wiringPiNodeStruct *node, int pin)
{
	unsigned char spiData [3];
	unsigned char startBit = 0x01;
	unsigned char chanBits;
	int chan = pin - node->pinBase;
	if (chan == 0)
		chanBits = 0b10100000;
	else
		chanBits = 0b11100000;
	spiData[0] = startBit;
	spiData[1] = chanBits;
	spiData[2] = 0;

	wiringPiSPIDataRW(node->fd,spiData,3);

	return ((spiData[1]&0xF)<<8)|(spiData[2]) & 0xFFF;
}

int MCP3202Setup(const int pinBase, int spiChannel, int spiFreq) {
	struct wiringPiNodeStruct *node;

	if (wiringPiSPISetup(spiChannel,spiFreq) < 0)
		return FALSE;

	node = wiringPiNewNode(pinBase,4);
	node->fd = spiChannel;
	node->analogRead=myAnalogRead;

	return TRUE;
}
/*unsigned int sampleADC(int fd, int chan){
	unsigned char spiData[2];
	spiData[0] = 0b11010000;
	spiData[1] = 0;

	wiringPiSPIDataRW(fd,spiData,1);

	return ((spiData[0]<<8)|spiData[1]) & 0x4FF;
}*/
int main(){
	float vref = 4.6;
	const int SPIChan = 0;
	const int SPIFreq = 2000000;
	wiringPiSetup();
	/*if (wiringPiSPISetup(0,2000000) < 0){
		perror("SPI Setup Error");
		exit(errno);
	}*/
	MCP3202Setup(ADC_PINBASE,SPIChan,SPIFreq);
	unsigned int sample;
	float convertedVal;
	while (1){
		delay(50);
		//sample = sampleADC(0,0);
		sample = analogRead(ADC1_PIN);
		convertedVal = (float)sample*vref/4096;
		printf("\rSample: %d <----> %0.4f V",sample,convertedVal);
		fflush(stdout);
	}
	return 0;
}
