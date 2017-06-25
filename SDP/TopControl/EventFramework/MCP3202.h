/* MCP3202.h
 * Underwater Drone/ROV
 */

#ifndef MCP3202_H
#define MCP3202_H

#define ADC_PINBASE 65
#define ADC1_PIN    65
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <wiringPi.h>

int MCP3202Setup(const int pinBase, int spiChannel, int spiFreq);

#endif
