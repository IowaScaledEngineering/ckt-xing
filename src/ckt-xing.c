/*************************************************************************
Title:    CKT-TINYBELL Railroad Crossing Bell Circuit
Authors:  Nathan D. Holmes <maverick@drgw.net>
          Based on the work of David Johnson-Davies - www.technoblogy.com - 23rd October 2017
           and used under his Creative Commons Attribution 4.0 International license
File:     $Id: $
License:  GNU General Public License v3

CREDIT:
    The basic idea behind this playback design came from David Johson-Davies, who
    provided the basic framework and the place where I started.

LICENSE:
    Copyright (C) 2019 Michael Petersen, Nathan Holmes, with portions from 
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
#include <stdbool.h>

// 16kHz 8 bit unsigned PCM data
#include "tinybell.h"

// playBell acts as an indicator between the main loop and 
// the ISR to tell it to shut down playback once it hits the end
// of the bell sound sample


volatile uint8_t stateTimer = 0;
volatile uint8_t playBell = 0;

inline void disableAmplifier()
{
	PORTB &= ~_BV(PB0);
}

inline void enableAmplifier()
{
	PORTB |= _BV(PB0);
}

inline uint8_t isAmplifierEnabled()
{
	return (PORTB & _BV(PB0));
}

#define LIGHT_LEFT_ON   0x01
#define LIGHT_RIGHT_ON  0x02
#define LIGHT_CONST_ON  0x04

#define LIGHT_RATE      6

volatile uint8_t lights = 0;
volatile uint8_t decisecs = 0;

volatile uint8_t readSensors = 0;

// 16kHz interrupt to load high speed PWMs
ISR(TIMER0_COMPA_vect) 
{
	static uint16_t wavIdx = 0;
	static uint8_t counter = 0;
	static uint8_t lightPWM = 0;
	static uint8_t leftLightPWMSetting = 0;
	static uint8_t rightLightPWMSetting = 0;
	static uint8_t constLightPWMSetting = 0;
	static uint8_t ticks = 0;

	uint8_t tmp = 0;

	if (isAmplifierEnabled())
	{
		OCR1A = pgm_read_byte(&tinybell_16k_wav[wavIdx++]);
	
		if (wavIdx == tinybell_16k_wav_len)
		{
			wavIdx = 0;
			// Make sure we only stop at the end of the bell sound, otherwise it may cut off
			// mid-cycle and sound very strange
			if (!playBell)
			{
				// Stop timer0 ISR
	//			TIMSK = 0;
				// Stop speaker output, center output
				OCR1A = 0x80;
				disableAmplifier();
			}
		}
	}

	// Now do light stuff
	
	lightPWM = ((lightPWM + 1) & 0x3F);

	if (lightPWM > leftLightPWMSetting)
		tmp |= _BV(PB4);
	if (lightPWM > rightLightPWMSetting)
		tmp |= _BV(PB5);
	if (lightPWM > constLightPWMSetting)
		tmp |= _BV(PB6);

	PORTB = (PORTB & ~(_BV(PB4) | _BV(PB5) | _BV(PB6))) | tmp;

	counter++;
	if (159 == counter) // Roughly 100Hz counter
	{
		counter = 0;
		if (10 == ++ticks)
		{
			ticks = 0;
			decisecs++;
			readSensors++;
			if (stateTimer)
				stateTimer--;
		}

		// Do light things
		if (lights & LIGHT_LEFT_ON)
		{
			if (leftLightPWMSetting < 0x3F)
				leftLightPWMSetting += LIGHT_RATE;

		} else {
			if (leftLightPWMSetting > 0)
				leftLightPWMSetting -= LIGHT_RATE;
		}

		// Do light things
		if (lights & LIGHT_RIGHT_ON)
		{
			if (rightLightPWMSetting < 0x3F)
				rightLightPWMSetting += LIGHT_RATE;

		} else {
			if (rightLightPWMSetting > 0)
				rightLightPWMSetting -= LIGHT_RATE;
		}

		// Do light things
		if (lights & LIGHT_CONST_ON)
		{
			if (constLightPWMSetting < 0x3F)
				constLightPWMSetting += LIGHT_RATE;

		} else {
			if (constLightPWMSetting > 0)
				constLightPWMSetting -= LIGHT_RATE;
		}

	}
}


#define STATE_IDLE                   0x00
#define STATE_ACTIVE_UNK             0x08

#define STATE_APPROACH_EASTBOUND     0x10
#define STATE_APPROACH_EAST_LOCKOUT  0x11
#define STATE_ACTIVE_EASTBOUND       0x12
#define STATE_SHUTDOWN_EASTBOUND     0x13
#define STATE_LOCKOUT_EASTBOUND      0x14

#define STATE_APPROACH_WESTBOUND     0x20
#define STATE_APPROACH_WEST_LOCKOUT  0x21
#define STATE_ACTIVE_WESTBOUND       0x22
#define STATE_SHUTDOWN_WESTBOUND     0x23
#define STATE_LOCKOUT_WESTBOUND      0x24

#define DETECTOR_APPRCH_EAST         0x01
#define DETECTOR_ISLAND_EAST         0x02
#define DETECTOR_ISLAND_WEST         0x04
#define DETECTOR_APPRCH_WEST         0x08

#define EAST_APPROACH_TIMEOUT             100
#define WEST_APPROACH_TIMEOUT             100

#define EAST_DEPARTURE_TIMEOUT             10
#define WEST_DEPARTURE_TIMEOUT             10

#define EAST_LOCKOUT_TIMER                100
#define WEST_LOCKOUT_TIMER                100

#define ISLAND_TIMER                       20


#define   TMD26711_ADDR   0x39
#define   INFO_ADDR       0x20

#define   PROXIMITY_THRESHOLD     0x300
#define   SENSOR_ERROR_THRESHOLD  0
#define   PPULSE_DEFAULT          8

#define   ON_DEBOUNCE_DEFAULT   1


#define   SCL     PA2

#define   SDA_1   PA4
#define   SDA_2   PA5
#define   SDA_3   PA6
#define   SDA_4   PA7

#define ON_DEBOUNCE_DEFAULT     1
#define OFF_DEBOUNCE_DEFAULT    20


static void sda_low() 
{
	PORTA &= ~(_BV(SDA_1) | _BV(SDA_2) | _BV(SDA_3) | _BV(SDA_4));
	DDRA |= _BV(SDA_1) | _BV(SDA_2) | _BV(SDA_3) | _BV(SDA_4);
	_delay_us(10);
}

static void sda_high() 
{
	DDRA &= ~(_BV(SDA_1) | _BV(SDA_2) | _BV(SDA_3) | _BV(SDA_4));
	PORTA |= (_BV(SDA_1) | _BV(SDA_2) | _BV(SDA_3) | _BV(SDA_4));
	_delay_us(10);
}
static void scl_low() { PORTA &= ~(_BV(SCL)); _delay_us(10); }
static void scl_high() { PORTA |= _BV(SCL); _delay_us(10); }

void i2cStart()
{
	scl_high();
	sda_low();
	scl_low();
	sda_high();
}

void i2cStop()
{
	scl_low();
	sda_low();
	scl_high();
	sda_high();
}

uint8_t i2cWriteByte(uint8_t byte)
{
	uint8_t i = 0x80, ack = 0x00;

	do
	{
		if(byte & i)
			sda_high();
		else
			sda_low();
		
		scl_high();
		scl_low();
		
		i >>= 1;
	} while(i);

	sda_high();  // Release SDA
	scl_high();

	if(!(PINA & _BV(SDA_1)))
		ack |= 0x01;
	if(!(PINA & _BV(SDA_2)))
		ack |= 0x02;
	if(!(PINA & _BV(SDA_3)))
		ack |= 0x04;
	if(!(PINA & _BV(SDA_4)))
		ack |= 0x08;

	scl_low();

	return ack;
}

void i2cReadByte(uint8_t ack, uint8_t* data)
{
	uint8_t i;

	for(i=0; i<4; i++)
		data[i] = 0;

	for(i=0; i<8; i++)
	{
		data[0] <<= 1;
		data[1] <<= 1;
		data[2] <<= 1;
		data[3] <<= 1;

		scl_high();
		if(PINA & _BV(SDA_1))
			data[0] |= 0x01;
		if(PINA & _BV(SDA_2))
			data[1] |= 0x01;
		if(PINA & _BV(SDA_3))
			data[2] |= 0x01;
		if(PINA & _BV(SDA_4))
			data[3] |= 0x01;
		scl_low();
	}
	
	if(ack)
		sda_low();
	scl_high();
	scl_low();
	sda_high();
}

uint8_t writeByte(uint8_t addr, uint8_t cmd, uint8_t writeVal)
{
	uint8_t ack;
	
	i2cStart();
	
	i2cWriteByte(addr << 1);
	i2cWriteByte(cmd);
	ack = i2cWriteByte(writeVal);

	i2cStop();

	return ack;
}

uint8_t readWord(uint8_t addr, uint8_t cmd, uint16_t wdata[])
{
	uint8_t ack = 0x0F, i;
	uint8_t data[4];
	
	i2cStart();
	
	ack &= i2cWriteByte(addr << 1);
	ack &= i2cWriteByte(cmd);

	i2cStart();

	ack &= i2cWriteByte((addr << 1) | 0x01);
	i2cReadByte(1, data);
	for(i=0; i<4; i++)
		wdata[i] = data[i];

	i2cReadByte(0, data);
	for(i=0; i<4; i++)
		wdata[i] |= ((uint16_t)data[i])<<8;
	
	i2cStop();

	return ack;
}

void initializeTMD26711()
{
	// Initialize TMD26711 (bit 0x80 set to indicate command)
	writeByte(TMD26711_ADDR, 0x80|0x00, 0x00);   // Start with everything disabled
	writeByte(TMD26711_ADDR, 0x80|0x01, 0xFF);   // Minimum ATIME
	writeByte(TMD26711_ADDR, 0x80|0x02, 0xFF);   // Maximum integration time
	writeByte(TMD26711_ADDR, 0x80|0x03, 0xFF);   // Minimum wait time
	
	// Note: IRQ not currently used
	writeByte(TMD26711_ADDR, 0x80|0x08, 0x00);   // Set interrupt low threshold to 0x0000
	writeByte(TMD26711_ADDR, 0x80|0x09, 0x00);
	writeByte(TMD26711_ADDR, 0x80|0x0A, 0x00);   // Set interrupt low threshold to 0x0300
	writeByte(TMD26711_ADDR, 0x80|0x0B, 0x03);
	writeByte(TMD26711_ADDR, 0x80|0x0C, 0x10);   // Single out-of-range cycle triggers interrupt

	writeByte(TMD26711_ADDR, 0x80|0x0D, 0x00);   // Long wait disabled
	writeByte(TMD26711_ADDR, 0x80|0x0E, PPULSE_DEFAULT); // Pulse count
	writeByte(TMD26711_ADDR, 0x80|0x0F, 0x28);   // 100% LED drive strength, 4x gain, Use channel 1 diode (ch 1 seems less sensitive to fluorescent) light)

	writeByte(TMD26711_ADDR, 0x80|0x00, 0x27);   // Power ON, Enable proximity, Enable proximity interrupt (not used currently)
}

uint8_t readTMD26711s()
{
	static uint8_t sensorError[4] = {0,0,0,0};
	static bool detect[4] = {false, false, false, false};
	static uint8_t count[4] = {0, 0, 0, 0};
	uint16_t proximity[4];
	uint8_t ack = 0, i;

	uint8_t retval = 0;

	writeByte(TMD26711_ADDR, 0x80|0x0E, PPULSE_DEFAULT);

	if (sensorError[0] || sensorError[1] || sensorError[2] || sensorError[3])
		initializeTMD26711();

	ack = readWord(TMD26711_ADDR, 0x80|0x20|0x18, proximity);  // Read data register (0x80 = command, 0x20 = auto-increment)

	// Check for missing ACKs, which would indicate a sensor malfunction
	for(i=0; i<4; i++)
	{
		if (0 == (ack & _BV(i)))
		{
			// Sensor's gone wonky, reset it and try again
			if (sensorError[i] < 255)
				sensorError[i]++;

			if (sensorError[i] > SENSOR_ERROR_THRESHOLD)
			{
				detect[i] = false;
				proximity[i] = 0;
				count[i] = 0;
			}

			// This sensor didn't answer, disregard it for now
			continue;
		}

		sensorError[i] = 0;


		if(!detect[i] && (proximity[i] >= PROXIMITY_THRESHOLD))
		{
			// ON debounce
			if(++count[i] > ON_DEBOUNCE_DEFAULT)
			{
				detect[i] = true;
				count[i] = 0;
			}
		}
		else if( (!detect[i] && (proximity[i] < PROXIMITY_THRESHOLD)) 
			|| (detect[i] && (proximity[i] >= PROXIMITY_THRESHOLD)) )
		{
			count[i] = 0;
		}
		else if(detect[i] && (proximity[i] < PROXIMITY_THRESHOLD))
		{
			// OFF debounce
			if(++count[i] > OFF_DEBOUNCE_DEFAULT)
			{
				detect[i] = false;
				count[i] = 0;
			}
		}

		if (detect[i])
			retval |= _BV(i);
	}

	return retval;
}


int main(void)
{
	uint8_t crossingState = STATE_IDLE;
	bool activateCrossing = false;
	volatile uint8_t detectors = 0;

	// Deal with watchdog first thing
	MCUSR = 0;								// Clear reset status
	wdt_reset();                     // Reset the WDT, just in case it's still enabled over reset
	wdt_enable(WDTO_1S);             // Enable it at a 1S timeout.

	cli();
	// This will be a 3.3V system running on the PLL clock, which will feed in a 16MHz input clock
	// The fuses are set to do a prescalar of 8 so we don't exceed the maximum 3.3V speed of ~12 MHz
	// We need to bump this back to a 2x prescalar to get 8MHz of operating speed
	CLKPR = _BV(CLKPCE);  // First enable changing the prescalar
	CLKPR = _BV(CLKPS0);  // Set prescalar to 2x


	// Enable 64 MHz PLL and use as source for Timer1
	PLLCSR = _BV(PCKE) | _BV(PLLE);


	// Set up Timer/Counter1 for PWM output on PB1 (OCR1A)
	TCCR1A = _BV(PWM1A) | _BV(COM1A1);            // PWM A, clear on match
	TCCR1B = _BV(CS10);									 // Run Timer1 at 1:1 prescale off 64MHz PCLK
//	TCCR1D = _BV(WGM10);  // This puts it in phase/freq correct PWM mode - probably don't want
	OCR1A = 0x80;                                 // 50% duty at start

	// Set up Timer/Counter0 for 16kHz interrupt to output samples.
	TIMSK = _BV(OCIE0A);                          // Timer interrupts OFF
	TCCR0A = _BV(CTC0);                           // Set Timer 0 to Clear Timer on Compare Match mode
	TCCR0B = _BV(CS01);                           // 1/8 prescale
	OCR0A = 63;                                   // Get as close to a 16kHz rate as we can


	DDRB |= _BV(PB0) | _BV(PB1) | _BV(PA4) | _BV(PA5) | _BV(PA6);        // Left, Right, Constant outputs
	PORTB |= _BV(PB3) | _BV(PA4) | _BV(PA5) | _BV(PA6); // Enable pullup

	PORTA = _BV(SCL) | _BV(SDA_1) | _BV(SDA_2) | _BV(SDA_3) | _BV(SDA_4);  // SCL high, along with pullups on SDA1-4
	DDRA |= _BV(SCL) | _BV(PA3);

	disableAmplifier();                           // Disable the amplifier until it's needed to save power

	initializeTMD26711();

	sei();

	while(1)
	{
		wdt_reset();

		activateCrossing = false;

		if (readSensors >= 1)
		{
			PORTA |= _BV(PA3);
			detectors = readTMD26711s();
			PORTA &= ~(_BV(PA3));
			readSensors = 0;
		}


		switch(crossingState)
		{
			case STATE_IDLE:
				//  Lights off, bell off
  				//  If island, go to STATE_ACTIVE_UNK
				//  If east approach, set approach_timer and go to STATE_APPROACH_EASTBOUND
				//  If west approach, set approach_timer and go to STATE_APPROACH_WESTBOUND
				if (detectors & (DETECTOR_ISLAND_EAST | DETECTOR_ISLAND_WEST))
				{
					crossingState = STATE_ACTIVE_UNK;
					stateTimer = ISLAND_TIMER;
				}
				else if (detectors & DETECTOR_APPRCH_EAST)
				{
					crossingState = STATE_APPROACH_EASTBOUND;
					stateTimer = EAST_APPROACH_TIMEOUT;
				}
				else if (detectors & DETECTOR_APPRCH_WEST)
				{
					crossingState = STATE_APPROACH_WESTBOUND;
					stateTimer = WEST_APPROACH_TIMEOUT;
				}
				else
				{
					activateCrossing = false;
				}

				break;

			case STATE_ACTIVE_UNK:
				// PURPOSE: Turns on the grade crossing if the island sensors suddenly get hit
				// Lights on, bell on
				// If island, go to STATE_ACTIVE_UNK
				// If !island, goto STATE_IDLE
				if (!(detectors & (DETECTOR_ISLAND_EAST | DETECTOR_ISLAND_WEST)) && 0 == stateTimer)
					crossingState = STATE_IDLE;
				activateCrossing = true;
				break;


			case STATE_APPROACH_EASTBOUND:
				// PURPOSE: Activates crossing when the east approach sensor is triggered
				//  and starts a timer, by which point the train must have hit the island
				// Lights on, bell on
				// If island, go to STATE_ACTIVE_EASTBOUND
				// If approach_timer expired, goto STATE_APPROACH_EAST_LOCKOUT
				if (detectors & (DETECTOR_ISLAND_EAST | DETECTOR_ISLAND_WEST))
					crossingState = STATE_ACTIVE_EASTBOUND;

				if (0 == stateTimer)
				{
					crossingState = STATE_APPROACH_EAST_LOCKOUT;
				}
				activateCrossing = true;

				break;  

			case STATE_APPROACH_EAST_LOCKOUT:
				// PURPOSE: Waits for the east approach detector to drop out after we've 
				//    timed out on eastbound approach
				// Lights off, bell off
				// If island, go to STATE_ACTIVE_EASTBOUND
				// If !east approach, goto STATE_IDLE
				// If west approach, goto STATE_APPROACH_WESTBOUND
				if (detectors & (DETECTOR_ISLAND_EAST | DETECTOR_ISLAND_WEST))
					crossingState = STATE_ACTIVE_EASTBOUND;

				if (!(detectors & (DETECTOR_APPRCH_EAST)))
					crossingState = STATE_IDLE;

				if (detectors & (DETECTOR_APPRCH_WEST))
				{
					crossingState = STATE_APPROACH_WESTBOUND;
					stateTimer = WEST_APPROACH_TIMEOUT;
				}
				activateCrossing = false;

				break;

			case STATE_ACTIVE_EASTBOUND:
				// Lights on, bell on
				// If !island, go to STATE_SHUTDOWN_EASTBOUND
				activateCrossing = true;
				if (!(detectors & (DETECTOR_ISLAND_EAST | DETECTOR_ISLAND_WEST)))
				{
					crossingState = STATE_SHUTDOWN_EASTBOUND;
					stateTimer = EAST_DEPARTURE_TIMEOUT;
				}
  				break;

			case STATE_SHUTDOWN_EASTBOUND:
				// Lights on, bell on
				// If island, go to STATE_ACTIVE_EASTBOUND
				// If !island and shutdown_timer expired, set lockout_timer and go to STATE_LOCKOUT_EASTBOUND
				if (detectors & (DETECTOR_ISLAND_EAST | DETECTOR_ISLAND_WEST))
					crossingState = STATE_ACTIVE_EASTBOUND;
				else if (0 == stateTimer)
				{
					crossingState = STATE_LOCKOUT_EASTBOUND;
					stateTimer = EAST_LOCKOUT_TIMER;
				}

				activateCrossing = true;

  				break;

			case STATE_LOCKOUT_EASTBOUND:
				// This state prevents retriggering on the west approach sensor for an eastbound
				// Lights off, bell off
				// If island, go to STATE_ACTIVE_EASTBOUND
				// If lockout_timer expired, go to STATE_IDLE
				if (detectors & (DETECTOR_ISLAND_EAST | DETECTOR_ISLAND_WEST))
					crossingState = STATE_ACTIVE_EASTBOUND;
				else if (0 == stateTimer && !(detectors & DETECTOR_APPRCH_WEST))
				{
					crossingState = STATE_IDLE;
				}
				activateCrossing = false;
				break;

			default:
				crossingState = STATE_IDLE;
				break;

		}

		// Total Override from external enable pin
		if (0 == (PINB & _BV(PB3)))
			activateCrossing = true;

		if (activateCrossing)
		{
			playBell = 1;
			enableAmplifier();
			if (decisecs & 0x08)
				lights = LIGHT_LEFT_ON | LIGHT_CONST_ON;
			else
				lights = LIGHT_RIGHT_ON | LIGHT_CONST_ON;
		} else {
			playBell = 0;
			lights = 0;
		}
	}
	
}

