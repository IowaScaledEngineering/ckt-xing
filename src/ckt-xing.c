/*************************************************************************
Title:    CKT-TINYBELL Railroad Crossing Bell Circuit
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

CREDIT:
    The basic idea behind this playback design came from David Johson-Davies, who
    provided the basic framework and the place where I started.

LICENSE:
    Copyright (C) 2021 Michael Petersen and Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <stdlib.h>
#include <stdbool.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "debouncer.h"
#include "hardware.h"


#define LIGHT_LEFT_ON   0x01
#define LIGHT_RIGHT_ON  0x02
#define LIGHT_CONST_ON  0x04
#define LIGHT_RATE      6

volatile uint8_t lights = 0;
volatile uint8_t decisecs = 0;
volatile uint8_t eventTriggers = 0;
volatile uint8_t lightFlashCounter = 0;

void activateLights()
{
	if (0 == lights)
		lights = LIGHT_LEFT_ON | LIGHT_CONST_ON;
}

void deactivateLights()
{
	lights = 0;
}



#define EVENT_DO_INPUT_READ   0x01
#define EVENT_CHANGE_FLASHER  0x02

ISR(TIMER0_COMPA_vect) 
{
	static uint8_t counter = 0;
	static uint8_t ticks = 0;

	static uint8_t lightPWM = 0;
	static uint8_t leftLightPWMSetting = 0;
	static uint8_t rightLightPWMSetting = 0;
	static uint8_t constLightPWMSetting = 0;
	
	static uint8_t lightFlashCounter = 0;

	// Do the light PWM stuff first so there's no phase jitter that would be
	// visually distracting
	uint8_t tmp = 0;
	
	lightPWM = ((lightPWM + 1) & 0x3F);
	
	if (lightPWM < leftLightPWMSetting)
		tmp |= _BV(PB2);
	if (lightPWM < rightLightPWMSetting)
		tmp |= _BV(PB1);
	if (lightPWM < constLightPWMSetting)
		tmp |= _BV(PB0);

	PORTB = (PORTB & ~(_BV(PB2) | _BV(PB1) | _BV(PB0))) | tmp;

	if (159 == ++counter) // Roughly 100Hz counter
	{
		counter = 0;

		// Every 20mS, read inputs
		if (ticks & 0x01)
			eventTriggers |= EVENT_DO_INPUT_READ;

		if (10 == ++ticks)
		{
			// 10Hz timer
			ticks = 0;

			// If the flashing lights aren't on, reset the flasher counter
			if (0 == (lights & (LIGHT_LEFT_ON | LIGHT_RIGHT_ON)))
			{
				lightFlashCounter = 0;
			}
			else if (8 == ++lightFlashCounter)
			{
				// Else the flashing lights are on and 0.8s has elapsed, switch lights
				lightFlashCounter = 0;
				if (lights & LIGHT_LEFT_ON)
					lights = (lights & ~(LIGHT_LEFT_ON)) | LIGHT_RIGHT_ON;
				else if (lights & LIGHT_RIGHT_ON)
					lights = (lights & ~(LIGHT_RIGHT_ON)) | LIGHT_LEFT_ON;
			}
		}
		

		// Recalculate light states
		if (lights & LIGHT_LEFT_ON)
		{
			if (leftLightPWMSetting <= 0x3F)
				leftLightPWMSetting += LIGHT_RATE;

		} else {
			if (leftLightPWMSetting > 0)
				leftLightPWMSetting -= LIGHT_RATE;
		}

		// Do light things
		if (lights & LIGHT_RIGHT_ON)
		{
			if (rightLightPWMSetting <= 0x3F)
				rightLightPWMSetting += LIGHT_RATE;

		} else {
			if (rightLightPWMSetting > 0)
				rightLightPWMSetting -= LIGHT_RATE;
		}

		// Do light things
		if (lights & LIGHT_CONST_ON)
		{
			if (constLightPWMSetting <= 0x3F)
				constLightPWMSetting += LIGHT_RATE;

		} else {
			if (constLightPWMSetting > 0)
				constLightPWMSetting -= LIGHT_RATE;
		}
	}
}

inline void init_16kHz_timer0()
{
	TCNT0 = 0;
	OCR0A = 63;                                   // Get as close to a 16kHz rate as we can

	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS01);
	TIMSK0 |= _BV(OCIE0A);
}

void init()
{
	init_gpio();
	init_16kHz_timer0();
}


int main(void)
{
	DebounceState8_t confSwitches;
	DebounceState8_t xingInputs;
	
	// Deal with watchdog first thing
	MCUSR = 0;                       // Clear reset status
	wdt_reset();                     // Reset the WDT, just in case it's still enabled over reset
	wdt_enable(WDTO_1S);             // Enable it at a 1S timeout.
	cli();
	init();
	
	initDebounceState8(&confSwitches, getSwitchState());
	initDebounceState8(&xingInputs, getInputState());
	
	sei();

	while(1)
	{
		wdt_reset();

		if (eventTriggers & EVENT_DO_INPUT_READ)
		{
			eventTriggers &= ~(EVENT_DO_INPUT_READ);
			debounce8(getSwitchState(), &confSwitches);
			debounce8(getInputState(), &xingInputs);
		}

		if (xingInputs.debounced_state & INPUT_XING_ACTIVE)
		{
			activateLights();
			activateBell();
		}
		else
		{
			deactivateLights();
			deactivateBell();
		}
	}
}

