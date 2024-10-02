/*************************************************************************
Title:    Crossing Track Detection State Machine 
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     crossingTrack.c
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

#include "crossingTrack.h"

void initializeCrossingTrack(CrossingTrack_t* t)
{
	initDebounceState8(&t->sensorDebouncer, 0);
	t->active = false;
	t->stateTimer = 0;
	t->trackState = DETECTION_STATE_IDLE;
	t->timeIslandTimeout = 10;
	t->timeApproachReset = 30;
	t->ledState = LIGHT_OFF;
	t->timeApproachTimeout = DECISECS_DETECTION_APPROACH_TIMEOUT;
	t->timeDetectionLockout = DECISECS_DETECTION_LOCKOUT_TIMEOUT;
}

bool isCrossingTrackActive(CrossingTrack_t* t)
{
	return t->active;
}

IndicatorLightState_t getStatusLEDState(CrossingTrack_t* t)
{
	return t->ledState;
}

void setCrossingMinimumOn(CrossingTrack_t* t, uint16_t decisecs)
{
	t->timeIslandTimeout = decisecs;
}

void setCrossingApproachTimeout(CrossingTrack_t* t, uint16_t decisecs)
{
	t->timeApproachTimeout = decisecs;
}

void setCrossingDetectionLockout(CrossingTrack_t* t, uint16_t decisecs)
{
	t->timeDetectionLockout = decisecs;
}

// This should be called at roughly 40Hz
void runCrossingTrackStateMachine(CrossingTrack_t* t, uint8_t xingInputs, bool isDecisecTick)
{
	debounce8(xingInputs, &t->sensorDebouncer);
	xingInputs = getDebouncedState(&t->sensorDebouncer);

	if (isDecisecTick && t->stateTimer > 0)
		t->stateTimer--;

	switch(t->trackState)
	{
		case DETECTION_STATE_IDLE:
			// Crossing circuit is completely idle, no lockouts or timeouts
			t->active = false;
			t->ledState = LIGHT_OFF;
			t->stateTimer = 0;
			
			if ((INPUT_WEST_APPR_OCC & xingInputs))
				t->trackState = DETECTION_STATE_WEST_APPROACH_SETUP;
			else if ((INPUT_EAST_APPR_OCC & xingInputs))
				t->trackState = DETECTION_STATE_EAST_APPROACH_SETUP;
			else if ((INPUT_ISLAND_OCC & xingInputs))
				t->trackState = DETECTION_STATE_ISLAND_SURPRISE; // Train showed up out of nowhere on the island
			break;

		case DETECTION_STATE_ISLAND_SURPRISE:
			// A train from an unknown direction has shown up on the island circuit.  Since it's on the
			// island, get detection going immediately
			t->active = true;
			t->ledState = LIGHT_ON;
			if((INPUT_ISLAND_OCC & xingInputs))
				t->stateTimer = t->timeIslandTimeout;
			else if (0 == t->stateTimer)
			{
				t->trackState = DETECTION_STATE_IDLE;
			}
			break;


		// *********************************************************************************
		// State Machine Cases for Approaching from the WEST APPROACH DETECTOR
		// *********************************************************************************


		case DETECTION_STATE_WEST_APPROACH_SETUP:
			// Train has appeared on the west approach circuit
			t->active = true;
			t->ledState = LIGHT_ON;
			t->stateTimer = t->timeApproachTimeout;
			t->trackState = DETECTION_STATE_WEST_APPROACH;
			break;

		case DETECTION_STATE_WEST_APPROACH:
			// A train has tripped the west approach sensor and now we're waiting for it to either
			// hit the island circuit or time out
			t->active = true;
			t->ledState = LIGHT_FAST_BLINK;
			if ((INPUT_ISLAND_OCC & xingInputs))
			{
				t->stateTimer = 0;
				t->trackState = DETECTION_STATE_WEST_ACTIVE;
			}
			else if (0 == t->stateTimer)
			{
				// Timed out before hitting island circuit
				t->trackState = DETECTION_STATE_WEST_TIMEOUT;
			}
			break;

		case DETECTION_STATE_WEST_ACTIVE:
			// A train from the west has entered the island circuit
			t->active = true;
			t->ledState = LIGHT_ON;
			if((INPUT_ISLAND_OCC & xingInputs))
				t->stateTimer = t->timeIslandTimeout;
			else if (0 == t->stateTimer)
			{
				t->active = false;
				
				// If the train has already hit the east approach detector, lock it out until it clears
				//  otherwise, use a time-based mechanism to keep it from retriggering
				if (INPUT_EAST_APPR_OCC & xingInputs)
					t->trackState = DETECTION_STATE_WEST_DET_LOCKOUT_SETUP;
				else
					t->trackState = DETECTION_STATE_WEST_TIME_LOCKOUT_SETUP;
			}
			break;

		case DETECTION_STATE_WEST_DET_LOCKOUT_SETUP:
			// A train from the west has cleared the island circuit for a minimum of DETECTION_STATE_MIN_ON_TIME
			// AND has already tripped the east detector.  Now we need to lock out the east detector
			// until it goes off and stays off for ~5 seconds
			// This state sets that up for moving into DETECTION_STATE_WEST_DET_LOCKOUT_SETUP
			t->active = false;
			t->ledState = LIGHT_DOUBLE_SLOW_BLINK;
			t->stateTimer = t->timeApproachReset;
			t->trackState = DETECTION_STATE_WEST_DET_LOCKOUT;
			break;

		case DETECTION_STATE_WEST_DET_LOCKOUT:
			// A train from the west has cleared the island circuit for a minimum of DETECTION_STATE_MIN_ON_TIME
			// and now we need to lock out the east detector for some length of time
			// This state waits until either we see the island go active again or until the east detector drops for
			// a minimum amount of time.
			if (INPUT_EAST_APPR_OCC & xingInputs)
				t->stateTimer = t->timeApproachReset;

			if((INPUT_ISLAND_OCC & xingInputs))
				t->trackState = DETECTION_STATE_WEST_ACTIVE;
			else if (0 == t->stateTimer && !(INPUT_EAST_APPR_OCC & xingInputs))
				t->trackState = DETECTION_STATE_IDLE;
			break;


		case DETECTION_STATE_WEST_TIME_LOCKOUT_SETUP:
			// A train from the west has cleared the island circuit for a minimum of DETECTION_STATE_MIN_ON_TIME
			// and now we need to lock out the east detector for some length of time
			// This state sets that up for moving into DETECTION_STATE_WEST_LOCKOUT
			t->active = false;
			t->ledState = LIGHT_SLOW_BLINK;
			t->stateTimer = t->timeDetectionLockout;
			t->trackState = DETECTION_STATE_WEST_TIME_LOCKOUT;
			break;

		case DETECTION_STATE_WEST_TIME_LOCKOUT:
			// A train from the west has cleared the island circuit for a minimum of DETECTION_STATE_MIN_ON_TIME
			// and now we need to lock out the east detector for some length of time
			// This state waits until either we see the island go active again or until the timeout has lapsed and
			// the east side detector is inactive
			t->active = false;
			t->ledState = LIGHT_SLOW_BLINK;
			if((INPUT_ISLAND_OCC & xingInputs))
				t->trackState = DETECTION_STATE_WEST_ACTIVE;
			else if (0 != t->stateTimer && (INPUT_EAST_APPR_OCC & xingInputs))
				t->trackState = DETECTION_STATE_WEST_DET_LOCKOUT_SETUP;
			else if (0 == t->stateTimer && !(INPUT_EAST_APPR_OCC & xingInputs))
				t->trackState = DETECTION_STATE_IDLE;
			break;

		case DETECTION_STATE_WEST_TIMEOUT:
			// This case is where the west approach triggered, but the island was not reached 
			// within the timeout interval.
			// If we see the island trip, go into west active
			// Otherwise, if west approach clears to go back to idle
			if ((INPUT_ISLAND_OCC & xingInputs))
				t->trackState = DETECTION_STATE_WEST_ACTIVE;
			else if (!(INPUT_WEST_APPR_OCC & xingInputs))
				t->trackState = DETECTION_STATE_IDLE;
			else
				t->active = false;
			break;


		// *********************************************************************************
		// State Machine Cases for Approaching from the EAST APPROACH DETECTOR
		// *********************************************************************************

		case DETECTION_STATE_EAST_APPROACH_SETUP:
			// Train has appeared on the east approach circuit
			t->active = true;
			t->ledState = LIGHT_FAST_BLINK;
			t->stateTimer = t->timeApproachTimeout;
			t->trackState = DETECTION_STATE_EAST_APPROACH;
			break;

		case DETECTION_STATE_EAST_APPROACH:
			// A train has tripped the east approach sensor and now we're waiting for it to either
			// hit the island circuit or time out
			t->active = true;
			t->ledState = LIGHT_FAST_BLINK;
			if ((INPUT_ISLAND_OCC & xingInputs))
			{
				t->stateTimer = 0;
				t->trackState = DETECTION_STATE_EAST_ACTIVE;
			}
			else if (0 == t->stateTimer)
			{
				// Timed out before hitting island circuit
				t->trackState = DETECTION_STATE_EAST_TIMEOUT;
			}
			break;

		case DETECTION_STATE_EAST_ACTIVE:
			// A train from the east has entered the island circuit
			t->active = true;
			t->ledState = LIGHT_ON;
			// As long as something's in the island, keep resetting the timer
			if((INPUT_ISLAND_OCC & xingInputs))
				t->stateTimer = t->timeIslandTimeout;
			else if (0 == t->stateTimer)
			{
				t->active = false;

				if (INPUT_WEST_APPR_OCC & xingInputs)
					t->trackState = DETECTION_STATE_EAST_DET_LOCKOUT_SETUP;
				else
					t->trackState = DETECTION_STATE_EAST_TIME_LOCKOUT_SETUP;
			}
			break;
			
		case DETECTION_STATE_EAST_DET_LOCKOUT_SETUP:
			// A train from the east has cleared the island circuit for a minimum of DETECTION_STATE_MIN_ON_TIME
			// AND has already tripped the west detector.  Now we need to lock out the west detector
			// until it goes off and stays off for ~5 seconds
			// This state sets that up for moving into DETECTION_STATE_WEST_DET_LOCKOUT_SETUP
			t->active = false;
			t->ledState = LIGHT_DOUBLE_SLOW_BLINK;
			t->stateTimer = t->timeApproachReset;
			t->trackState = DETECTION_STATE_EAST_DET_LOCKOUT;
			break;

		case DETECTION_STATE_EAST_DET_LOCKOUT:
			// A train from the east has cleared the island circuit for a minimum of DETECTION_STATE_MIN_ON_TIME
			// and now we need to lock out the west detector for some length of time
			// This state waits until either we see the island go active again or until the west detector drops for
			// a minimum amount of time.
			t->active = false;
			t->ledState = LIGHT_DOUBLE_SLOW_BLINK;
			if (INPUT_WEST_APPR_OCC & xingInputs)
				t->stateTimer = t->timeApproachReset;

			if((INPUT_ISLAND_OCC & xingInputs))
				t->trackState = DETECTION_STATE_EAST_ACTIVE;
			else if (0 == t->stateTimer && !(INPUT_WEST_APPR_OCC & xingInputs))
				t->trackState = DETECTION_STATE_IDLE;
			break;

		case DETECTION_STATE_EAST_TIME_LOCKOUT_SETUP:
			// A train from the west has cleared the island circuit for a minimum of DETECTION_STATE_MIN_ON_TIME
			// and now we need to lock out the east detector for some length of time
			// This state sets that up for moving into DETECTION_STATE_WEST_LOCKOUT
			t->active = false;
			t->ledState = LIGHT_SLOW_BLINK;
			t->stateTimer = t->timeDetectionLockout;
			t->trackState = DETECTION_STATE_EAST_TIME_LOCKOUT;
			break;

		case DETECTION_STATE_EAST_TIME_LOCKOUT:
			// A train from the east has cleared the island circuit for a minimum of DETECTION_STATE_MIN_ON_TIME
			// and now we need to lock out the east detector for some length of time
			// This state waits until either we see the island go active again or until the timeout has lapsed and
			// the west side detector is inactive
			t->active = false;
			t->ledState = LIGHT_SLOW_BLINK;
			if((INPUT_ISLAND_OCC & xingInputs))
				t->trackState = DETECTION_STATE_EAST_ACTIVE;
			else if (0 != t->stateTimer && (INPUT_WEST_APPR_OCC & xingInputs))
				t->trackState = DETECTION_STATE_EAST_DET_LOCKOUT_SETUP;
			else if (0 == t->stateTimer && !(INPUT_WEST_APPR_OCC & xingInputs))
				t->trackState = DETECTION_STATE_IDLE;
			break;

		case DETECTION_STATE_EAST_TIMEOUT:
			// This case is where the east approach triggered, but the island was not reached 
			// within the timeout interval.
			// If we see the island trip, go into east active
			// Otherwise, if east approach clears to go back to idle
			if ((INPUT_ISLAND_OCC & xingInputs))
				t->trackState = DETECTION_STATE_EAST_ACTIVE;
			else if (!(INPUT_EAST_APPR_OCC & xingInputs))
				t->trackState = DETECTION_STATE_IDLE;
			else
				t->active = false;
			break;

		default:
			t->trackState = DETECTION_STATE_IDLE;
			break;
	}
}

