/* MCP3002.h
 * Underwater Drone/ROV
 */

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <stdlib.h>
#include <stdio.h>
#include "MCP3202.h"
#include <errno.h>

static int myAnalogRead (struct wiringPiNodeStruct *node, int pin){
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

	return (((spiData[1]&0xF)<<8)|spiData[2]) & 0xFFF;
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
