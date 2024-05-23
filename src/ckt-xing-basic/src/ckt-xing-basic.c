/*************************************************************************
Title:    CKT-XING-BASIC
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
          Based on the work of David Johnson-Davies - www.technoblogy.com - 23rd October 2017
           and used under his Creative Commons Attribution 4.0 International license
File:     $Id: $
License:  GNU General Public License v3

CREDIT:
    The basic idea behind this playback design came from David Johson-Davies, who
    provided the basic framework and the place where I started.

LICENSE:
    Copyright (C) 2024 Michael Petersen, Nathan Holmes, with portions from 
     David Johson-Davies under a Creative Commons Attribution 4.0 license

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <stdbool.h>
#include <stdint.h>
#include "spiflash.h"
#include "audioAndLights.h"
#include "ispl.h"
#include "debouncer.h"

#define MAX_TRACKS 6

DebounceState8_t inputDebouncer;

void readInputs()
{
	static uint32_t lastRead = 0;
	uint8_t currentInputState = 0;

	if (millis > lastRead + 10)
	{
		lastRead = millis;

		// Inputs (bit / io / name):  
		//  0 - PA4 - Enable 1
		//  1 - PB0 - SW1
		//  2 - PB2 - SW2
		//  3 - PB4 - SW3
		//  4 - PB5 - SW4
		//  5 - PB6 - SW5
		currentInputState = ~(((PINA & _BV(PA4))>>4) | ((PINB & _BV(PB0))<<1) | (PINB & _BV(PB2)) | ((PINB & (_BV(PB4) | _BV(PB5) | _BV(PB6)))>>1));
		debounce8(currentInputState, &inputDebouncer);
	} 
}

bool recentlyUsed(uint8_t trackNum)
{
	static uint8_t lastPlayed[3] = { 0xFF, 0xFF, 0xFF };

	for(uint8_t i=0; i<sizeof(lastPlayed); i++)
	{
		if (lastPlayed[i] == trackNum)
			return true;
	}

	lastPlayed[2] = lastPlayed[1];
	lastPlayed[1] = lastPlayed[0];
	lastPlayed[0] = trackNum;
	return false;
}

typedef enum 
{
	CROSSING_OFF,
	CROSSING_START,
	CROSSING_ACTIVE,
	CROSSING_WAIT,
	CROSSING_SHUTDOWN
}
CrossingState;

#define INPUT_ACTIVE   0x01

uint8_t getBellNumber(uint8_t switches)
{
	uint8_t bn = 0;

	switches >>= 3;
	// The switch bits are in backward order - SW5 is the LSB, SW3 is the MSB
	if (switches & 0x04)
		bn |= 1;
	if (switches & 0x02)
		bn |= 2;
	if (switches & 0x01)
		bn |= 4;

	bn = min(6, bn);
	return bn;
}

uint32_t getTimeoutMillis(uint8_t switches)
{
	switch(0x03 & (switches>>1))
	{
		case 0:
		default:
			return (5 * 1000);

		case 2:
			return (10 * 1000);

		case 1:
			return (15 * 1000);

		case 3:
			return (30 * 1000);
	}
}

int main(void)
{
	// Deal with watchdog first thing
	MCUSR = 0;              // Clear reset status
	wdt_reset();            // Reset the WDT, just in case it's still enabled over reset
	wdt_enable(WDTO_1S);    // Enable it at a 1S timeout.

	// PORT A
	//  PA7 - Output - /SHUTDOWN to amplifier
	//  PA6 - Output - Left Crossing Lamp
	//  PA5 - Output - Right Crossing Lamp
	//  PA4 - Input  - /EN1x (enable pullup)	
	//  PA3 - Output - /CS to flash
	//  PA2 - Output - CLK to flash
	//  PA1 - Output - MOSI to flash
	//  PA0 - Input - MISO to flash (enable pullup)

	// PORT B
	//  PB7 - n/a    - /RESET (not I/O pin)
	//  PB6 - Input  - SW5 (enable pullup)
	//  PB5 - Input  - SW4 (enable pullup)
	//  PB4 - Input  - SW3 (enable pullup)
	//  PB3 - Output - Audio PWM Output
	//  PB2 - Input  - SW2 (enable pullup)
	//  PB1 - Output - (AVR programming SPI)
	//  PB0 - Input  - SW1 (enable pullup)


	PORTA = 0b00011001;
	DDRA  = 0b11101110;

	PORTB = 0b11110101;     // Just make everything low
	DDRB  = 0b00001010;     // And set it as an output

	initDebounceState8(&inputDebouncer, 0x00);

	audioInitialize();
	spiSetup();
	spiflashReset();

	CrossingState crossingState = CROSSING_OFF;
	uint32_t crossingTimeoutStartMillis = 0, crossingTimeout = 0;
	uint8_t playTrack = 0;
	AudioAssetRecord r;

	sei();
	wdt_reset();

	isplInitialize(); // What should I do if this fails?

	while(1)
	{
		wdt_reset();
		uint8_t inputState = getDebouncedState(&inputDebouncer);

		switch(crossingState)
		{
			case CROSSING_OFF:
				deactivateLights();
				if (inputState & INPUT_ACTIVE)
				{
					isplAudioAssetLoad(getBellNumber(inputState), &r);
					crossingState = CROSSING_START;
				}
				break;


			case CROSSING_START:
				activateLights();
				audioPlay(r.addr, r.size, r.sampleRate, true);
				crossingState = CROSSING_ACTIVE;
				break;


			case CROSSING_ACTIVE:
				if (!(inputState & INPUT_ACTIVE))
				{
					crossingState = CROSSING_WAIT;
					crossingTimeoutStartMillis = millis;
					crossingTimeout = getTimeoutMillis(inputState);
				}
				break;

			case CROSSING_WAIT:
				if (inputState & INPUT_ACTIVE)
					crossingState = CROSSING_ACTIVE;

				if (millis - crossingTimeoutStartMillis > crossingTimeout)
					crossingState = CROSSING_SHUTDOWN;
				break;

			case CROSSING_SHUTDOWN:
				deactivateLights();
				stopAudioRepeat();
				crossingState = CROSSING_OFF;
				break;


			default:
				crossingState = CROSSING_OFF;
				break;

		}
		audioPump();
		readInputs();
	}
}




