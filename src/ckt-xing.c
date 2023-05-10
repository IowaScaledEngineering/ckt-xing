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
#include "time-config.h"

#define LIGHT_LEFT_ON   0x01
#define LIGHT_RIGHT_ON  0x02
#define LIGHT_CONST_ON  0x04
#define LIGHT_NOFADE    0x80

#define LIGHT_RATE      4

#define min(a,b)  ((a<b)?(a):(b))

volatile uint8_t lights = 0;
volatile uint8_t decisecs = 0;
volatile uint8_t eventTriggers = 0;

#define LIGHT_FLAGS (LIGHT_LEFT_ON | LIGHT_RIGHT_ON | LIGHT_CONST_ON )

void activateLights()
{
	if (0 == (lights & LIGHT_FLAGS))
		lights |= LIGHT_LEFT_ON | LIGHT_CONST_ON;
}

void deactivateLights()
{
	lights &= ~(LIGHT_FLAGS);
}



#define EVENT_DO_INPUT_READ   0x01
#define EVENT_DECISEC_TICK    0x02

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
			eventTriggers |= EVENT_DECISEC_TICK;

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
		

		if (lights & LIGHT_NOFADE)
		{
			// LED mode, lights on/off instantly
			leftLightPWMSetting = (lights & LIGHT_LEFT_ON)?0x3F:0;
			rightLightPWMSetting = (lights & LIGHT_RIGHT_ON)?0x3F:0;
			constLightPWMSetting = (lights & LIGHT_CONST_ON)?0x3F:0;
			
		} else {
			// Lamp mode, lights fade in/out
			if (lights & LIGHT_LEFT_ON)
			{
				if (leftLightPWMSetting <= 0x3F)
					leftLightPWMSetting += LIGHT_RATE;

			} else {
				if (leftLightPWMSetting > 0)
					leftLightPWMSetting -= min(leftLightPWMSetting, LIGHT_RATE);
			}

			// Do light things
			if (lights & LIGHT_RIGHT_ON)
			{
				if (rightLightPWMSetting <= 0x3F)
					rightLightPWMSetting += LIGHT_RATE;

			} else {
				if (rightLightPWMSetting > 0)
					rightLightPWMSetting -= min(rightLightPWMSetting, LIGHT_RATE);
			}

			// Do light things
			if (lights & LIGHT_CONST_ON)
			{
				if (constLightPWMSetting <= 0x3F)
					constLightPWMSetting += LIGHT_RATE;

			} else {
				if (constLightPWMSetting > 0)
					constLightPWMSetting -= min(constLightPWMSetting, LIGHT_RATE);
			}
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

void initialTestBlink()
{
	return;
	
}

bool getIslandOccupied(DebounceState8_t* xingInputs)
{
	if (xingInputs->debounced_state & INPUT_ISLAND_OCC)
		return true;
	return false;
}

bool getWestApproachOccupied(DebounceState8_t* xingInputs)
{
	if (xingInputs->debounced_state & INPUT_WEST_APPR_OCC)
		return true;
	return false;
}

bool getEastApproachOccupied(DebounceState8_t* xingInputs)
{
	if (xingInputs->debounced_state & INPUT_EAST_APPR_OCC)
		return true;
	return false;
}

bool getCrossingActive(DebounceState8_t* xingInputs)
{
	if (xingInputs->debounced_state & INPUT_XING_ACTIVE)
		return true;
	return false;
}
/*
// Activity state machine

 - DETECTION_STATE_IDLE 
  
  * On WOCC, go to DETECTION_STATE_WEST_APPR
  * On EOCC, go to DETECTION_STATE_EAST_APPR
  * On IOCC, go to DETECTION_STATE_ISLAND_SURPRISE

 - DETECTION_STATE_ISLAND_SURPRISE
 * Set state timer to 2 seconds 
 * if timer expired and !IOCC, go to DETECTION_STATE_IDLE

*/

typedef enum
{
	DETECTION_STATE_IDLE = 0,
	DETECTION_STATE_ISLAND_SURPRISE,
	DETECTION_STATE_WEST_APPROACH_SETUP,
	DETECTION_STATE_WEST_APPROACH,
	DETECTION_STATE_WEST_ACTIVE,
	DETECTION_STATE_WEST_LOCKOUT_SETUP,
	DETECTION_STATE_WEST_LOCKOUT,
	DETECTION_STATE_WEST_TIMEOUT,

	DETECTION_STATE_EAST_APPROACH_DELAY_SETUP,
	DETECTION_STATE_EAST_APPROACH_DELAY,
	DETECTION_STATE_EAST_APPROACH_SETUP,
	DETECTION_STATE_EAST_APPROACH,
	DETECTION_STATE_EAST_ACTIVE,
	DETECTION_STATE_EAST_LOCKOUT_SETUP,
	DETECTION_STATE_EAST_LOCKOUT,
	DETECTION_STATE_EAST_TIMEOUT,
} DetectionState_t;  

typedef enum
{
	GATES_DISABLED            = 0,
	GATES_2Q_ONLY             = 1,
	GATES_4Q_SIMULTANEOUS_UP  = 2,
	GATES_4Q_DELAYED_UP       = 3
} GateConfiguration_t;

GateConfiguration_t getGateConfiguration(DebounceState8_t* switchInputs)
{
	return (GateConfiguration_t)((switchInputs->debounced_state & 0x30)>>4);
}

bool getBellOnDropOnlyConfiguration(DebounceState8_t* switchInputs)
{
	return (switchInputs->debounced_state & 0x40)?true:false;
}


typedef enum
{
	SIGNAL_STATE_IDLE = 0,
	SIGNAL_STATE_START_SETUP,
	SIGNAL_STATE_START,
	SIGNAL_STATE_MAIN_GATE_DOWN_SETUP,
	SIGNAL_STATE_MAIN_GATE_DOWN,
	SIGNAL_STATE_4Q_GATE_WAIT_SETUP,
	SIGNAL_STATE_4Q_GATE_WAIT,
	SIGNAL_STATE_4Q_GATE_DOWN_SETUP,
	SIGNAL_STATE_4Q_GATE_DOWN,
	SIGNAL_STATE_4Q_GATE_UP_SETUP,
	SIGNAL_STATE_4Q_GATE_UP,
	SIGNAL_STATE_MAIN_GATE_UP_SETUP,
	SIGNAL_STATE_MAIN_GATE_UP,
	SIGNAL_STATE_ACTIVE,
	SIGNAL_STATE_ACTIVE_2Q,
	SIGNAL_STATE_ACTIVE_4Q,
} SignalState_t;  


void runSignalStateMachine(DebounceState8_t* xingInputs, DebounceState8_t* switchInputs, bool decrementStateTimer)
{
	static SignalState_t state = SIGNAL_STATE_IDLE;
	static uint8_t stateTimer = 0;

	GateConfiguration_t gateConf = getGateConfiguration(switchInputs);
	bool active = getCrossingActive(xingInputs);
	bool gatesEnabled = (gateConf != GATES_DISABLED)?true:false;
	bool bellOnGateDropOnly = getBellOnDropOnlyConfiguration(switchInputs) && gatesEnabled;

	if (decrementStateTimer && stateTimer > 0)
		stateTimer--;
	
	switch(state)
	{
		case SIGNAL_STATE_IDLE:
			if (active)
				state = SIGNAL_STATE_START_SETUP;
			else
			{
				deactivateLights();
				deactivateBell();
				deactivateMainGates();
				deactivateSecondaryGates();
			}
			break;
		
		case SIGNAL_STATE_START_SETUP:
			activateLights();
			activateBell();
			deactivateMainGates();
			deactivateSecondaryGates();
			// If gates are enabled at all
			stateTimer = (gatesEnabled)?DECISECS_SIGNAL_MAIN_GATE_DELAY:0;
			state = SIGNAL_STATE_START;
			break;

		case SIGNAL_STATE_START:
			// Lights and bell are started
			activateLights();
			activateBell();

			// If gates are enabled, wait the SIGNAL_STATE_MAIN_GATE_DELAY interval before lowering them
			if (!active)
			{
				state = SIGNAL_STATE_IDLE;
				break;
			}
			
			if (0 == stateTimer)
			{
				// We've waited for the lights to flash long enough, lower the gates
				if (!gatesEnabled)
					state = SIGNAL_STATE_ACTIVE;
				else
					state = SIGNAL_STATE_MAIN_GATE_DOWN_SETUP;
			}
			break;

		case SIGNAL_STATE_MAIN_GATE_DOWN_SETUP:
			// This state, we're starting the main gates down and waiting for them to fall in SIGNAL_STATE_MAIN_GATE_DOWN
			if (!active)
			{
				state = SIGNAL_STATE_IDLE;
				break;
			}
			activateLights();
			activateBell();
			activateMainGates();
			deactivateSecondaryGates();
			stateTimer = DECISECS_SIGNAL_MAIN_GATE_DROP;
			state = SIGNAL_STATE_MAIN_GATE_DOWN;
			break;

		case SIGNAL_STATE_MAIN_GATE_DOWN:
			activateMainGates();
			deactivateSecondaryGates();
			
			if (!active)
			{
				// We went inactive while the gates were going down
				state = SIGNAL_STATE_MAIN_GATE_UP_SETUP;
			} else if (0 == stateTimer) {
				// We've now waited long enough for the gates to fall, go either for 4Q gates or active
				// depending on configuration
				if (gateConf == GATES_4Q_DELAYED_UP || gateConf == GATES_4Q_SIMULTANEOUS_UP)
					state = SIGNAL_STATE_4Q_GATE_WAIT_SETUP;
				else
					state = SIGNAL_STATE_ACTIVE_2Q;
			}
			break;

		case SIGNAL_STATE_4Q_GATE_WAIT_SETUP:
			activateMainGates();
			activateLights();
			activateBell();
			if (!active)
				state = SIGNAL_STATE_MAIN_GATE_UP_SETUP;
			else
			{
				stateTimer = DECISECS_SIGNAL_4Q_GATE_DELAY;
				state = SIGNAL_STATE_4Q_GATE_WAIT;
			}
			break;

		case SIGNAL_STATE_4Q_GATE_WAIT:
			activateMainGates();
			activateLights();
			activateBell();

			if (!active)
				state = SIGNAL_STATE_MAIN_GATE_UP_SETUP;
			else if (0 == stateTimer)
				state = SIGNAL_STATE_4Q_GATE_DOWN_SETUP;
			break;

		case SIGNAL_STATE_4Q_GATE_DOWN_SETUP:
			activateMainGates();
			activateLights();
			activateBell();
			if (!active)
				state = SIGNAL_STATE_MAIN_GATE_UP_SETUP;
			else
			{
				stateTimer = DECISECS_SIGNAL_4Q_GATE_DROP;
				state = SIGNAL_STATE_4Q_GATE_DOWN;
			}
			break;
			
		case SIGNAL_STATE_4Q_GATE_DOWN:
			activateMainGates();
			activateSecondaryGates();
			activateLights();
			activateBell();
			if (!active)
				state = SIGNAL_STATE_4Q_GATE_UP_SETUP;
			else if (0 == stateTimer)
			{
				state = SIGNAL_STATE_ACTIVE_4Q;
			}
			break;
			
		case SIGNAL_STATE_ACTIVE_2Q:
			activateMainGates();
			deactivateSecondaryGates();
			activateLights();
			if (bellOnGateDropOnly)
				deactivateBell();
			else
				activateBell();

			if (!active)
				state = SIGNAL_STATE_MAIN_GATE_UP_SETUP;
			break;

		case SIGNAL_STATE_ACTIVE_4Q:
			activateMainGates();
			activateSecondaryGates();
			activateLights();
			if (bellOnGateDropOnly)
				deactivateBell();
			else
				activateBell();

			if (!active)
				state = SIGNAL_STATE_4Q_GATE_UP_SETUP;
				
			break;
		
			
		case SIGNAL_STATE_ACTIVE:
			deactivateMainGates();
			deactivateSecondaryGates();
			activateLights();
			if (bellOnGateDropOnly)
				deactivateBell();
			else
				activateBell();

			if (!active)
				state = SIGNAL_STATE_IDLE;
			break;

		case SIGNAL_STATE_4Q_GATE_UP_SETUP:
			deactivateSecondaryGates();
			
			if (gateConf == GATES_4Q_DELAYED_UP)
			{
				stateTimer = DECISECS_SIGNAL_4Q_GATE_RISE_DELAY;
			} else {
				stateTimer = 0;
				deactivateMainGates();
			}
			state = SIGNAL_STATE_4Q_GATE_UP;
			break;

		case SIGNAL_STATE_4Q_GATE_UP:
			deactivateSecondaryGates();
			if (active)
			{
				// Go back to active
				state = SIGNAL_STATE_MAIN_GATE_DOWN_SETUP;
			} else if (0 == stateTimer) {
				state = SIGNAL_STATE_MAIN_GATE_UP_SETUP;
			}
			break;

		case SIGNAL_STATE_MAIN_GATE_UP_SETUP:
			deactivateMainGates();
			deactivateSecondaryGates();
			if (gatesEnabled)
			{
				stateTimer = DECISECS_SIGNAL_GATE_RISE;
			} else {
				stateTimer = 0;
			}
			state = SIGNAL_STATE_MAIN_GATE_UP;
			break;
			
		case SIGNAL_STATE_MAIN_GATE_UP:
			deactivateMainGates();
			deactivateSecondaryGates();
			if (0 == stateTimer)
				state = SIGNAL_STATE_IDLE;
			break;
		
		default:
			state = SIGNAL_STATE_IDLE;
			break;
	}
}


// SW8 - LED vs. Lamp
// SW7 - Bell always vs only on lowering
// SW6 - Gates 1
// SW5 - Gates 2
//   Gates 00 - No gates
//   Gates 01 - 2Q gates
//   Gates 10 - 4Q gates, simultaneous rise
//   Gates 11 - 4Q gates, delayed rise

// SW4 - 
// SW3 - Approach timeout 2
// SW2 - Approach timeout 1
// SW1 - Approach timeout 0

//   AT 000 - 10 seconds
//   AT 001 - 15 seconds
//   AT 010 - 20 seconds
//   AT 011 - 25 seconds
//   AT 100 - 30 seconds
//   AT 101 - 35 seconds
//   AT 110 - 40 seconds 
//   AT 111 - Reserved (test mode)




// Start - lights, gates 2s later
// Bell on or off once gates down


uint16_t getApproachTimeoutDecisecs(DebounceState8_t* confSwitches)
{
	uint8_t approachTimeoutSwitches = confSwitches->debounced_state & 0x07;
	uint16_t approachTimeoutDecisecs = 200;

	switch(approachTimeoutSwitches)
	{
		case 0:
			approachTimeoutDecisecs = 100;
			break;
		case 1:
			approachTimeoutDecisecs = 150;
			break;
		case 2:
			approachTimeoutDecisecs = 200;
			break;
		case 3:
			approachTimeoutDecisecs = 250;
			break;
		case 4:
			approachTimeoutDecisecs = 300;
			break;
		case 5:
			approachTimeoutDecisecs = 350;
			break;
		case 6:
			approachTimeoutDecisecs = 400;
			break;
		case 7:
		default:
			break;
	}
	return approachTimeoutDecisecs;
}

void runDetectionStateMachine(DebounceState8_t* xingInputs, DebounceState8_t* confSwitches, bool decrementStateTimer)
{
	static DetectionState_t detState = DETECTION_STATE_IDLE;
	static uint16_t stateTimer = 0;

	if (decrementStateTimer && stateTimer > 0)
		stateTimer--;

	switch(detState)
	{
		case DETECTION_STATE_IDLE:
			// Crossing circuit is completely idle, no lockouts or timeouts
			clearDetectionActive();
			stateTimer = 0;
			
			if (getWestApproachOccupied(xingInputs))
				detState = DETECTION_STATE_WEST_APPROACH_SETUP;
			else if (getEastApproachOccupied(xingInputs))
				detState = DETECTION_STATE_EAST_APPROACH_DELAY_SETUP;
			else if (getIslandOccupied(xingInputs))
				detState = DETECTION_STATE_ISLAND_SURPRISE; // Train showed up out of nowhere on the island
			break;

		case DETECTION_STATE_ISLAND_SURPRISE:
			// A train from an unknown direction has shown up on the island circuit.  Since it's on the
			// island, get detection going immediately
			setDetectionActive();
			if(getIslandOccupied(xingInputs))
				stateTimer = DECISECS_DETECTION_MINIMUM_ON;
			else if (0 == stateTimer)
			{
				detState = DETECTION_STATE_IDLE;
			}
			break;


		// *********************************************************************************
		// State Machine Cases for Approaching from the WEST APPROACH DETECTOR
		// *********************************************************************************


		case DETECTION_STATE_WEST_APPROACH_SETUP:
			// Train has appeared on the west approach circuit
			setDetectionActive();
			stateTimer = getApproachTimeoutDecisecs(confSwitches);
			detState = DETECTION_STATE_WEST_APPROACH;
			break;

		case DETECTION_STATE_WEST_APPROACH:
			// A train has tripped the west approach sensor and now we're waiting for it to either
			// hit the island circuit or time out
			setDetectionActive();
			if (getIslandOccupied(xingInputs))
			{
				stateTimer = 0;
				detState = DETECTION_STATE_WEST_ACTIVE;
			}
			else if (0 == stateTimer)
			{
				// Timed out before hitting island circuit
				detState = DETECTION_STATE_WEST_TIMEOUT;
			}
			break;

		case DETECTION_STATE_WEST_ACTIVE:
			// A train from the west has entered the island circuit
			setDetectionActive();
			if(getIslandOccupied(xingInputs))
				stateTimer = DECISECS_DETECTION_MINIMUM_ON;
			else if (0 == stateTimer)
			{
				clearDetectionActive();
				detState = DETECTION_STATE_WEST_LOCKOUT_SETUP;
			}
			break;

		case DETECTION_STATE_WEST_LOCKOUT_SETUP:
			// A train from the west has cleared the island circuit for a minimum of DETECTION_STATE_MIN_ON_TIME
			// and now we need to lock out the east detector for some length of time
			// This state sets that up for moving into DETECTION_STATE_WEST_LOCKOUT
			clearDetectionActive();
			stateTimer = DECISECS_DETECTION_LOCKOUT_TIMEOUT;
			detState = DETECTION_STATE_WEST_LOCKOUT;
			break;

		case DETECTION_STATE_WEST_LOCKOUT:
			// A train from the west has cleared the island circuit for a minimum of DETECTION_STATE_MIN_ON_TIME
			// and now we need to lock out the east detector for some length of time
			// This state waits until either we see the island go active again or until the timeout has lapsed and
			// the east side detector is inactive
			clearDetectionActive();
			if(getIslandOccupied(xingInputs))
				detState = DETECTION_STATE_WEST_ACTIVE;
			else if (0 == stateTimer && !getEastApproachOccupied(xingInputs))
				detState = DETECTION_STATE_IDLE;
			break;

		case DETECTION_STATE_WEST_TIMEOUT:
			// This case is where the west approach triggered, but the island was not reached 
			// within the timeout interval.
			// If we see the island trip, go into west active
			// Otherwise, if west approach clears to go back to idle


			if (getIslandOccupied(xingInputs))
				detState = DETECTION_STATE_WEST_ACTIVE;
			else if (!getWestApproachOccupied(xingInputs))
				detState = DETECTION_STATE_IDLE;
			else
				clearDetectionActive();
			break;


		// *********************************************************************************
		// State Machine Cases for Approaching from the EAST APPROACH DETECTOR
		// *********************************************************************************

		case DETECTION_STATE_EAST_APPROACH_DELAY_SETUP:
			stateTimer = 170;
			detState = DETECTION_STATE_EAST_APPROACH_DELAY;
			break;

		case DETECTION_STATE_EAST_APPROACH_DELAY:
			if (getIslandOccupied(xingInputs))
			{
				stateTimer = 0;
				detState = DETECTION_STATE_EAST_ACTIVE;
			}
			else if (0 == stateTimer)
			{
				detState = DETECTION_STATE_EAST_APPROACH_SETUP;
			}
			break;


		case DETECTION_STATE_EAST_APPROACH_SETUP:
			// Train has appeared on the east approach circuit
			setDetectionActive();
			stateTimer = getApproachTimeoutDecisecs(confSwitches);
			detState = DETECTION_STATE_EAST_APPROACH;
			break;

		case DETECTION_STATE_EAST_APPROACH:
			// A train has tripped the east approach sensor and now we're waiting for it to either
			// hit the island circuit or time out
			setDetectionActive();
			if (getIslandOccupied(xingInputs))
			{
				stateTimer = 0;
				detState = DETECTION_STATE_EAST_ACTIVE;
			}
			else if (0 == stateTimer)
			{
				// Timed out before hitting island circuit
				detState = DETECTION_STATE_EAST_TIMEOUT;
			}
			break;

		case DETECTION_STATE_EAST_ACTIVE:
			// A train from the east has entered the island circuit
			setDetectionActive();
			// As long as something's in the island, keep resetting the timer
			if(getIslandOccupied(xingInputs))
				stateTimer = DECISECS_DETECTION_MINIMUM_ON;
			else if (0 == stateTimer)
			{
				clearDetectionActive();
				detState = DETECTION_STATE_EAST_LOCKOUT_SETUP;
			}
			break;

		case DETECTION_STATE_EAST_LOCKOUT_SETUP:
			// A train from the west has cleared the island circuit for a minimum of DETECTION_STATE_MIN_ON_TIME
			// and now we need to lock out the east detector for some length of time
			// This state sets that up for moving into DETECTION_STATE_WEST_LOCKOUT
			clearDetectionActive();
			stateTimer = DECISECS_DETECTION_LOCKOUT_TIMEOUT;
			detState = DETECTION_STATE_EAST_LOCKOUT;
			break;

		case DETECTION_STATE_EAST_LOCKOUT:
			// A train from the east has cleared the island circuit for a minimum of DETECTION_STATE_MIN_ON_TIME
			// and now we need to lock out the east detector for some length of time
			// This state waits until either we see the island go active again or until the timeout has lapsed and
			// the west side detector is inactive
			clearDetectionActive();
			if(getIslandOccupied(xingInputs))
				detState = DETECTION_STATE_EAST_ACTIVE;
			else if (0 == stateTimer && !getWestApproachOccupied(xingInputs))
				detState = DETECTION_STATE_IDLE;
			break;

		case DETECTION_STATE_EAST_TIMEOUT:
			// This case is where the east approach triggered, but the island was not reached 
			// within the timeout interval.
			// If we see the island trip, go into east active
			// Otherwise, if west approach clears to go back to idle
			if (getIslandOccupied(xingInputs))
				detState = DETECTION_STATE_EAST_ACTIVE;
			else if (!getEastApproachOccupied(xingInputs))
				detState = DETECTION_STATE_IDLE;
			else
				clearDetectionActive();
			break;

		default:
			detState = DETECTION_STATE_IDLE;
			break;
	}
	
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
	
	initialTestBlink();
	
	
	sei();

	while(1)
	{
		wdt_reset();

		if (eventTriggers & EVENT_DO_INPUT_READ)
		{
			eventTriggers &= ~(EVENT_DO_INPUT_READ);
			debounce8(getSwitchState(), &confSwitches);
			debounce8(getInputState(), &xingInputs);
			
			// Reset things that could be affected by configuration changes
			
			if (confSwitches.debounced_state & SWITCH_LED_MODE)
				lights |= LIGHT_NOFADE;
			else
				lights &= ~LIGHT_NOFADE;
		}
		
		bool decrementStateTimers = false;
		if (eventTriggers & EVENT_DECISEC_TICK)
		{
			eventTriggers &= ~EVENT_DECISEC_TICK;
			decrementStateTimers = true;
		}

		runDetectionStateMachine(&xingInputs, &confSwitches, decrementStateTimers);
		runSignalStateMachine(&xingInputs, &confSwitches, decrementStateTimers);
	}
}

