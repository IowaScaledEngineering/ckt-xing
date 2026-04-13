/*************************************************************************
Title:    Hardware Configuration for CKT-XING-ADV
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2026 Michael Petersen & Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include "systemHardware.h"
#include "crossingTrack.h"
#include "audioAndLights.h"

bool initTCA9555()
{
	// PORT 0
	//  P0.7 - Input  - /Track 2 Cancel
	//  P0.6 - Input  - /Track 2 East Approach
	//  P0.5 - Input  - /Track 2 Island
	//  P0.4 - Input  - /Track 2 West Approach
	//  P0.3 - Input  - /Track 1 Cancel
	//  P0.2 - Input  - /Track 1 East Approach
	//  P0.1 - Input  - /Track 1 Island
	//  P0.0 - Input  - /Track 1 West Approach

	// PORT 1
	//  P1.7 - Output - Aux Gate 2
	//  P1.6 - Output - Aux Gate 1
	//  P1.5 - Output - Main Gate 2
	//  P1.4 - Output - Main Gate 1
	//  P1.3 - Output - Config LED D (high activates)
	//  P1.2 - Output - Config LED C
	//  P1.1 - Output - Config LED B
	//  P1.0 - Output - Config LED A

	// Set TCA9555 direction registers
	// Direction:  1=input, 0=output
	writeByte(TCA9555_ADDR_000, TCA9555_GPDDR0, 0b11111111);
	writeByte(TCA9555_ADDR_000, TCA9555_GPDDR1, 0b00000000);
	writeByte(TCA9555_ADDR_000, TCA9555_GPOUT0, 0b00000000);
	writeByte(TCA9555_ADDR_000, TCA9555_GPOUT1, 0b11110000);
    return true;
}

bool initPCA9685()
{
	// Put oscillator to sleep - needed to change prescaler
	writeByte(PCA9685_ADR_0, PCA9685_REG_MODE_1, 0b00110001);
	writeByte(PCA9685_ADR_0, PCA9685_REG_MODE_2, 0b00000100);

	writeByte(PCA9685_ADR_0, PCA9685_REG_PRESCALE, PCA9685_VAL_50HZ_PRESCALE);

	_delay_us(500); // Let oscillator stabilize per section 7.3.1.1 of the datasheet

	// Turn off LEDs (set high)
	writeBytes(PCA9685_ADR_0, PCA9685_REG_ALL_ON_L, (uint8_t[]){0x00, 0x10, 0x00, 0x00}, 4);

	// 1 count = 4.88uS.  0 degress = 205 count, 90 degrees = 307 counts, 180 degrees = 410 counts
	// Just start in the center at 307
	writeBytes(PCA9685_ADR_0, PCA9685_REG_CH0_ON_L, (uint8_t[]){0x00, 0x00, 0x33, 0x01}, 4);
	writeBytes(PCA9685_ADR_0, PCA9685_REG_CH1_ON_L, (uint8_t[]){0x00, 0x00, 0x33, 0x01}, 4);
	writeBytes(PCA9685_ADR_0, PCA9685_REG_CH2_ON_L, (uint8_t[]){0x00, 0x00, 0x33, 0x01}, 4);
	writeBytes(PCA9685_ADR_0, PCA9685_REG_CH3_ON_L, (uint8_t[]){0x00, 0x00, 0x33, 0x01}, 4);

	// Wake back up
	writeByte(PCA9685_ADR_0, PCA9685_REG_MODE_1, 0b10100001);

	return true;
}


bool initSystemHardwareState(SystemHardwareState_t* state)
{
    bool retval = false;
	for(uint8_t i=0; i<CONFIG_LEDS; i++)
		state->configModeLamp[i] = false;
	state->configValueLamp = 0;
	state->mainGatesActive = false;
	state->auxGatesActive = false;
	initDebounceState8(&state->trackSensorDebouncer, 0x00);
    initDebounceState8(&state->extInDebouncer, 0x00);
	retval = initTCA9555();
	retval &= initPCA9685();

	for(uint8_t i=0; i<4; i++)
        state->servoPosition[i] = SERVO_180_DEGREES<<4;
    return retval;
}

void readInputs(SystemHardwareState_t* state)
{
    uint8_t i=0;

    // Inputs are active low, but state machines are 
    //  positive logic, hence inversion here
	if (readByte(TCA9555_ADDR_000, TCA9555_GPIN0, &i))
        debounce8(~i, &state->trackSensorDebouncer);

    debounce8((EXT_IN_PIN & _BV(EXT_IN))?0:EXT_IN_ACTIVE_MASK, &state->extInDebouncer);
    
}

uint8_t getTrackAState(SystemHardwareState_t* state)
{
    return 0x0F & getDebouncedState(&state->trackSensorDebouncer);
}

uint8_t getTrackBState(SystemHardwareState_t* state)
{
    return 0x0F & (getDebouncedState(&state->trackSensorDebouncer)>>4);
}

void setOutputs(SystemHardwareState_t* state, Configuration_t* config , bool driveGates)
{
	uint8_t i = 0;
	static bool currentLEDs[8];

    if (driveGates)
    {
        if (state->mainGatesActive)
        {
            i |= GPOUT1_MAIN_GATES_DOWN_MASK;
            state->servoPosition[0] = MIN(config->servoLimit[0], state->servoPosition[0] + config->servoRate[0]);
            state->servoPosition[1] = MIN(config->servoLimit[1], state->servoPosition[1] + config->servoRate[1]);
        }
        else
        {
            i |= GPOUT1_MAIN_GATES_UP_MASK; 
            state->servoPosition[0] = MAX(config->servoLimit[4], state->servoPosition[0] - config->servoRate[0]);
            state->servoPosition[1] = MAX(config->servoLimit[5], state->servoPosition[1] - config->servoRate[1]);
        }

        if (state->auxGatesActive)
        {
            i |= GPOUT1_AUX_GATES_DOWN_MASK;
            state->servoPosition[2] = MIN(config->servoLimit[2], state->servoPosition[2] + config->servoRate[2]);
            state->servoPosition[3] = MIN(config->servoLimit[3], state->servoPosition[3] + config->servoRate[3]);
        }
        else
        {
            i |= GPOUT1_AUX_GATES_UP_MASK; 
            state->servoPosition[2] = MAX(config->servoLimit[6], state->servoPosition[2] - config->servoRate[2]);
            state->servoPosition[3] = MAX(config->servoLimit[7], state->servoPosition[3] - config->servoRate[3]);
        }
    }
    audioPump();

	if (state->configValueLamp & 0x08)
		i |= GPOUT1_CONFIG_VAL_A_LED_MASK;
	if (state->configValueLamp & 0x04)
		i |= GPOUT1_CONFIG_VAL_B_LED_MASK;
	if (state->configValueLamp & 0x02)
		i |= GPOUT1_CONFIG_VAL_C_LED_MASK;
	if (state->configValueLamp & 0x01)
		i |= GPOUT1_CONFIG_VAL_D_LED_MASK;

	writeByte(TCA9555_ADDR_000, TCA9555_GPOUT1, i);

    writeServoPosition(0, state->servoPosition[0]);
	writeServoPosition(1, state->servoPosition[1]);
    audioPump();
    writeServoPosition(2, state->servoPosition[2]);
    writeServoPosition(3, state->servoPosition[3]);
    audioPump();

    // Now we do the LEDs
	for(i=0; i<8; i++)
	{
		// Only send LED updates if it changed.  Otherwise, leads to flicker
		if (currentLEDs[i] == state->configModeLamp[i])
			continue;
		writeByte(PCA9685_ADR_0, PCA9685_REG_CH6_ON_H + (4 * (7-i)), state->configModeLamp[i]?0x07:0x10);
    	audioPump();

		currentLEDs[i] = state->configModeLamp[i];
	}
}

bool isExtInActive(SystemHardwareState_t* state)
{
    return (EXT_IN_ACTIVE_MASK & getDebouncedState(&state->extInDebouncer))?true:false;
}

void writeServoPosition(uint8_t servoNum, uint16_t servoPosition)
{
	servoPosition = servoPosition >> 4;

    if (servoNum > 3)
        return;

    writeBytes(PCA9685_ADR_0, PCA9685_REG_CH0_OFF_L + servoNum*4, (uint8_t[]){servoPosition & 0xFF, (servoPosition>>8) & 0x0F}, 2);
}

uint8_t getSwitches()
{
	return ((SWITCH_UP_PIN & _BV(SWITCH_UP))?SWITCH_UP_MASK:0) 
		| ((SWITCH_DOWN_PIN & _BV(SWITCH_DOWN))?SWITCH_DOWN_MASK:0) 
		| ((SWITCH_NEXT_PIN & _BV(SWITCH_NEXT))?SWITCH_NEXT_MASK:0);
}
