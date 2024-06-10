/*************************************************************************
Title:    CKT-XING-BASIC Audio and Light Control
Authors:  Nathan Holmes <maverick@drgw.net>, Colorado, USA
          Michael Petersen <railfan@drgw.net>, Colorado, USA
File:     audioAndLights.c
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2024 Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License along 
    with this program. If not, see http://www.gnu.org/licenses/
    
*************************************************************************/
#include <string.h>
#include "audioAndLights.h"
#include "spiflash.h"
#include "util/atomic.h"

// Light Control Stuff
volatile uint8_t lightControl = 0;

AudioRingBuffer audioBuffer;
uint32_t audioStartIdx = 0;
uint32_t audioEndIdx = 0;
uint32_t audioReadIdx = 0;
uint32_t audioDataLen = 0;
bool audioLoop = false;

// Audio playing ISR
volatile uint32_t millis = 0;

void activateLights()
{
	lightControl |= LIGHT_ACTIVE;
}

void deactivateLights()
{
	lightControl &= ~(LIGHT_ACTIVE);
}

void stopAudioRepeat()
{
	audioLoop = false;
}


ISR(TIMER0_COMPA_vect) 
{
	static uint16_t micros = 0;
	static uint8_t next = 0x7F;
	static uint8_t lightPWMAdjustCounter = 0;
	static uint8_t lightFlashCounter = 0;
	static uint8_t lightPWM = 0;
	static uint8_t leftLightPWMSetting = 0;
	static uint8_t rightLightPWMSetting = 0;

	OCR1B = next;
	next = audioBufferPop();

	// Do this right up top so it doesn't increment on us
	micros += OCR0A;

	// Now do the light calculations
	uint8_t tmp = 0;
	lightPWM = ((lightPWM + 1) & 0x3F);

	if (lightPWM < leftLightPWMSetting)
		tmp |= _BV(PA5);
	if (lightPWM < rightLightPWMSetting)
		tmp |= _BV(PA6);

	PORTA = (PORTA & ~(_BV(PA5) | _BV(PA6))) | tmp;

	if(micros >= 1000)
	{
		millis++;
		micros -= 1000;

		// 100Hz (roughly) adjustments to light PWMs
		if (++lightPWMAdjustCounter >= 10)
		{
			lightPWMAdjustCounter = 0;

			if (++lightFlashCounter >= 80 
				|| ((lightControl & LIGHT_ACTIVE) && !(lightControl & (LIGHT_RIGHT_ON | LIGHT_LEFT_ON))))  // Handles startup case
			{
				lightFlashCounter = 0;
				if (lightControl & LIGHT_ACTIVE)
				{
					if (lightControl & LIGHT_LEFT_ON)
						lightControl = (lightControl & ~(LIGHT_LEFT_ON)) | LIGHT_RIGHT_ON;
					else
						lightControl = (lightControl & ~(LIGHT_RIGHT_ON)) | LIGHT_LEFT_ON;
				} else {
					lightControl &= ~(LIGHT_LEFT_ON | LIGHT_RIGHT_ON);
				}
			}

			if (lightControl & LIGHT_NOFADE)
			{
				leftLightPWMSetting = (lightControl & LIGHT_LEFT_ON)?0x3F:0;
				rightLightPWMSetting = (lightControl & LIGHT_RIGHT_ON)?0x3F:0;
			} else {
				// Lamp mode, lights fade in/out
				if (lightControl & LIGHT_LEFT_ON)
				{
					if (leftLightPWMSetting <= 0x3F)
						leftLightPWMSetting += LIGHT_RATE;
				} else {
					if (leftLightPWMSetting > 0)
						leftLightPWMSetting -= min(leftLightPWMSetting, LIGHT_RATE);
				}

				// Do light things
				if (lightControl & LIGHT_RIGHT_ON)
				{
					if (rightLightPWMSetting <= 0x3F)
						rightLightPWMSetting += LIGHT_RATE;

				} else {
					if (rightLightPWMSetting > 0)
						rightLightPWMSetting -= min(rightLightPWMSetting, LIGHT_RATE);
				}
			}
		}
	}

}

uint32_t getMillis()
{
	uint32_t retmillis;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
	{
		retmillis = millis;
	}

	return retmillis;
}

void audioAmplifierEnable(uint8_t enable)
{
	if (enable)
		PORTA |= _BV(PA7);
	else
		PORTA &= ~(_BV(PA7));
}
	
void audioInitialize()
{
	audioReadIdx = audioStartIdx = audioDataLen = 0;
	audioBufferInitialize();

	// Enable 64 MHz PLL and use as source for Timer1
	PLLCSR = _BV(PLLE);
	
	// Wait for PLL lock per datasheet
	do
	{
		_delay_us(100);
	}
	while (!(PLLCSR & _BV(PLOCK)));

	// Enable PCK once PLL lock established
	PLLCSR = _BV(PCKE) | _BV(PLLE);

	TIMSK = 0;                                    // Timer interrupts OFF

	// Set up Timer/Counter1 for PWM output on PB3 (OC1B)
	TCCR1A = 0b00100001; // PWM B, clear on match
	TCCR1B = 0b00000001; // No dead time, 1:1 prescale for counter 1
	OCR1B = 0x7F;        // 50% duty at start

	// Set up Timer/Counter0 for 1MHz clock, interrupts to output samples.
	//  OCR0A will be loaded as the number of us between samples
	TCCR0A = 0b00000001;  // CTC Mode
	TCCR0B = 0b00000010;  // Just CS01 - 1/8 prescale
	OCR0A = 42;
	TIMSK = _BV(OCIE0A);
	
	audioAmplifierEnable(1);
}

bool audioIsPlaying()
{
	if (audioBufferDepth() > 0 || audioReadIdx < audioStartIdx + audioDataLen)
		return true;
	return false;
}

void audioPlay(uint32_t addr, uint32_t len, uint16_t sampleRateHz, bool loop)
{
	// Load ring buffer with initial data
	uint8_t loadBytes = (sizeof(audioBuffer.buffer) - 1) - audioBufferDepth();
	audioStartIdx = audioReadIdx = addr;
	audioDataLen = len;
	
	// 8MHz / prescaler 8:1 / sample frequency
	OCR0A = (uint8_t)(((8000000UL / 8UL) + ((uint32_t)sampleRateHz - 1)) / sampleRateHz);
	spiflashReadToRingBuffer(audioReadIdx, loadBytes);
	audioReadIdx += loadBytes;
	audioEndIdx = audioStartIdx + audioDataLen;

	audioLoop = loop;
}

void audioPump()
{
	uint8_t bufferFree = sizeof(audioBuffer.buffer) - audioBufferDepth();
	uint32_t bytesRemaining = audioEndIdx - audioReadIdx;

	if (audioLoop && 0 == bytesRemaining)
	{
		audioReadIdx = audioStartIdx;
		bytesRemaining = audioEndIdx - audioReadIdx;
	}

	if ((bytesRemaining) &&  bufferFree > 32)
	{
		uint8_t bytesToRead = min(bufferFree-1, bytesRemaining);
		spiflashReadToRingBuffer(audioReadIdx, bytesToRead);
		audioReadIdx += bytesToRead;
	}
}

void audioBufferInitialize()
{
	audioBuffer.headIdx = audioBuffer.tailIdx = 0;
	audioBuffer.full = false;
}

uint8_t audioBufferSize()
{
	return sizeof(audioBuffer.buffer);
}

uint8_t audioBufferDepth()
{
	if(audioBuffer.full)
		return(audioBufferSize());

	return (uint8_t)(audioBuffer.headIdx - audioBuffer.tailIdx) % sizeof(audioBuffer.buffer);
}

void audioBufferPush(uint8_t data)
{
	audioBuffer.buffer[audioBuffer.headIdx] = data;

	if( ++audioBuffer.headIdx >= sizeof(audioBuffer.buffer) )
		audioBuffer.headIdx = 0;

	if (audioBuffer.headIdx == audioBuffer.tailIdx)
		audioBuffer.full = true;
}

uint8_t audioBufferPop()
{
	uint8_t retval = 0;
	if (0 == audioBufferDepth())
		return(0x7F);

	retval = audioBuffer.buffer[audioBuffer.tailIdx];

	if( ++audioBuffer.tailIdx >= sizeof(audioBuffer.buffer) )
		audioBuffer.tailIdx = 0;
	audioBuffer.full = false;

	return(retval);
}

