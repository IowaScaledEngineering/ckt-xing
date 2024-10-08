/*************************************************************************
Title:    CKT-XING-TXM Track Expansion Module
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2024 Michael Petersen & Nathan Holmes

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
#include <util/atomic.h>

#include <avr/sleep.h>
#include <stdbool.h>
#include <stdint.h>

#include "debouncer.h"
#include "hardware.h"
#include "crossingTrack.h"

#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))

volatile uint32_t millis = 0;

uint32_t getMillis()
{
	uint32_t retmillis;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
	{
		retmillis = millis;
	}
	return retmillis;
}

ISR(TIMER0_COMPA_vect) 
{
	millis++;
}

void initializeTimer()
{
	TIMSK = 0;                                    // Timer interrupts OFF
	// Set up Timer/Counter0 for 1kHz clock
	TCCR0A = 0b00000001;  // CTC Mode
	TCCR0B = 0b00000011;  // CS01/CS00 - 1:64 prescaler
	OCR0A = 125;           // 8MHz / 64 / 125 = 1kHz
	TIMSK = _BV(OCIE0A);
}


// Island timeouts

// XY
// 00  - 0.5s
// 01  - 1s
// 10  - 2s
// 11  - 5s



uint8_t getIslandTimeoutDecisecs(uint8_t switches)
{
	// Switch X - bit 1
	// Switch Y - bit 0
	switch(0x03 & (switches))
	{
		case 0:
		default:
			return 5;

		case 1:
			return 10;

		case 2:
			return 20;

		case 3:
			return 50;
	}
}

// Approach timeouts

// ABC
// 000  - 20s
// 001  - 25s
// 010  - 30s
// 011  - 35s
// 100  - 40s
// 101  - 50s
// 110  - 60s
// 111  - Test Mode

uint16_t getApproachTimeoutDecisecs(uint8_t switches)
{
	// Switch A - bit 4
	// Switch B - bit 3
	// Switch C - bit 2

	switch(0x07 & (switches>>2))
	{
		case 0:
			return 200;

		case 1:
			return 250;

		case 2:
			return 300;

		case 3:
			return 350;

		case 4:
			return 400;

		case 5:
			return 500;

		case 6:
			return 600;
			
		case 7:  // This is really test mode
		default:
			return 300;
	}
}



int main(void)
{
	DebounceState8_t optionsDebouncer;
	CrossingTrack_t trackA;
	CrossingTrack_t trackB;

	uint32_t lastReadTime = 0;
	uint32_t currentTime = 0;
	uint8_t decisecTick = 0;
	uint8_t ledPhase = 0;

	// Deal with watchdog first thing
	MCUSR = 0;              // Clear reset status
	wdt_reset();            // Reset the WDT, just in case it's still enabled over reset
	wdt_enable(WDTO_1S);    // Enable it at a 1S timeout.

	// PORT A
	//  PA7 - Input  - Approach Timeout A
	//  PA6 - Input  - Approach Timeout B
	//  PA5 - Input  - Approach Timeout C
	//  PA4 - Input  - Island Timeout X
	//  PA3 - Input  - Island Timeout Y
	//  PA2 - Input  - Track A Approach Left
	//  PA1 - Input  - Track A Island
	//  PA0 - Input  - Track A Approach Right

	// PORT B
	//  PB7 - n/a    - /RESET (not I/O pin)
	//  PB6 - Input  - Track B Approach Right
	//  PB5 - Input  - Track B Island
	//  PB4 - Input  - Track B Approach Left
	//  PB3 - Output - Enable output driver (active high)
	//  PB2 - Output - (none - ICSP SCK)
	//  PB1 - Output - Track A Active LED
	//  PB0 - Output - Track B Active LED

	PORTA = 0b11111111;
	DDRA  = 0b00000000;

	PORTB = 0b11110000;
	DDRB  = 0b00001111;

	initializeTimer();
	initializeOptions(&optionsDebouncer);
	initializeCrossingTrack(&trackA);
	initializeCrossingTrack(&trackB);
	setCrossingActiveOutput(false);
	
	sei();
	wdt_reset();

	while(1)
	{
		wdt_reset();

		currentTime = getMillis();

		// Because debouncing and such is built into option reading and the MSS library, only 
		//  run the updates every 10mS or so.

		if (((uint32_t)currentTime - lastReadTime) > 25)
		{
			uint8_t i = 0;
			uint16_t j = 0;
			lastReadTime = currentTime;
			decisecTick = (decisecTick+1) % 4;

			readOptions(&optionsDebouncer);

			j = getApproachTimeoutDecisecs(getDebouncedState(&optionsDebouncer));
			setCrossingApproachTimeout(&trackA, j);
			setCrossingApproachTimeout(&trackB, j);

			// Make the back side lockout 5s longer than the front timeout
			// Normally won't affect anything, given that both should be reset
			//  by the train continuing through 
			j += 50; 
			setCrossingDetectionLockout(&trackA, j);
			setCrossingDetectionLockout(&trackB, j);


			i = getIslandTimeoutDecisecs(getDebouncedState(&optionsDebouncer));
			setIslandTimeout(&trackA, i);
			setIslandTimeout(&trackB, i);

			i = getTrackAInputs();
			runCrossingTrackStateMachine(&trackA, i,  0 == decisecTick);

			i = getTrackBInputs();
			runCrossingTrackStateMachine(&trackB, i,  0 == decisecTick);

			if (0 == decisecTick)
			{
				// Only run this 10 times/second
				setStatusLEDs(ledPhase++, getStatusLEDState(&trackA), getStatusLEDState(&trackB));
				if (ledPhase >= 20)
					ledPhase = 0;
			}

			setCrossingActiveOutput(isCrossingTrackActive(&trackA) || isCrossingTrackActive(&trackB));
		}

	}
}




