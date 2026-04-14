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

#include "xingSignals.h"
#include "configuration.h"
#include "timings.h"

void initCrossingSignals(CrossingSignalState_t* xingState, GateConfiguration_t gateConfig, bool bellOnGateDropOnly)
{
    xingState->gateConfig = gateConfig;
    xingState->lightsActive = false;
    xingState->bellActive = false;
    xingState->mainGatesActive = false;
    xingState->auxGatesActive = false;
    xingState->signalState = SIGNAL_STATE_IDLE;
    xingState->bellOnGateDropOnly = (gateConfig == GATES_DISABLED)?false:bellOnGateDropOnly;
    xingState->stateTimer = 0;    
}

void runCrossingSignalStateMachine(CrossingSignalState_t* xingState, bool active, bool isDecisec)
{
	bool gatesEnabled = (xingState->gateConfig != GATES_DISABLED)?true:false;

	// stateTimer is in decisecs
	if (isDecisec &&  xingState->stateTimer > 0)
		xingState->stateTimer--;
	
	switch(xingState->signalState)
	{
		case SIGNAL_STATE_IDLE:
			if (active)
				xingState->signalState = SIGNAL_STATE_START_SETUP;
			else
			{
                xingState->lightsActive = false;
                xingState->bellActive = false;
                xingState->mainGatesActive = false;
                xingState->auxGatesActive = false;
			}
			break;
		
		case SIGNAL_STATE_START_SETUP:
            xingState->lightsActive = true;
            xingState->bellActive = true;
            xingState->mainGatesActive = false;
            xingState->auxGatesActive = false;
			// If gates are enabled at all
			xingState->stateTimer = (gatesEnabled && (GATES_2Q_NO_DELAY != xingState->gateConfig))?DECISECS_SIGNAL_MAIN_GATE_DELAY:0;
			xingState->signalState = SIGNAL_STATE_START;
			break;

		case SIGNAL_STATE_START:
			// Lights and bell are started
            xingState->lightsActive = true;
            xingState->bellActive = true;

			// If gates are enabled, wait the SIGNAL_STATE_MAIN_GATE_DELAY interval before lowering them
			if (!active)
			{
				xingState->signalState = SIGNAL_STATE_IDLE;
				break;
			}
			
			if (0 == xingState->stateTimer)
			{
				// We've waited for the lights to flash long enough, lower the gates
				if (!gatesEnabled)
					xingState->signalState = SIGNAL_STATE_ACTIVE;
				else
					xingState->signalState = SIGNAL_STATE_MAIN_GATE_DOWN_SETUP;
			}
			break;

		case SIGNAL_STATE_MAIN_GATE_DOWN_SETUP:
			// This state, we're starting the main gates down and waiting for them to fall in SIGNAL_STATE_MAIN_GATE_DOWN
			if (!active)
			{
				xingState->signalState = SIGNAL_STATE_IDLE;
				break;
			}
            xingState->lightsActive = true;
            xingState->bellActive = true;
            xingState->mainGatesActive = true;
            xingState->auxGatesActive = false;
			xingState->stateTimer = DECISECS_SIGNAL_MAIN_GATE_DROP;
			xingState->signalState = SIGNAL_STATE_MAIN_GATE_DOWN;
			break;

		case SIGNAL_STATE_MAIN_GATE_DOWN:
            xingState->mainGatesActive = true;
            xingState->auxGatesActive = false;
			
			if (!active)
			{
				// We went inactive while the gates were going down
				xingState->signalState = SIGNAL_STATE_MAIN_GATE_UP_SETUP;
			} else if (0 == xingState->stateTimer) {
				// We've now waited long enough for the gates to fall, go either for 4Q gates or active
				// depending on configuration
				if (xingState->gateConfig == GATES_4Q_DELAYED_UP || xingState->gateConfig == GATES_4Q_SIMULTANEOUS_UP)
					xingState->signalState = SIGNAL_STATE_4Q_GATE_WAIT_SETUP;
				else
					xingState->signalState = SIGNAL_STATE_ACTIVE_2Q;
			}
			break;

		case SIGNAL_STATE_4Q_GATE_WAIT_SETUP:
            xingState->mainGatesActive = true;
            xingState->lightsActive = true;
            xingState->bellActive = true;
			if (!active)
				xingState->signalState = SIGNAL_STATE_MAIN_GATE_UP_SETUP;
			else
			{
				xingState->stateTimer = DECISECS_SIGNAL_4Q_GATE_DELAY;
				xingState->signalState = SIGNAL_STATE_4Q_GATE_WAIT;
			}
			break;

		case SIGNAL_STATE_4Q_GATE_WAIT:
            xingState->mainGatesActive = true;
            xingState->lightsActive = true;
            xingState->bellActive = true;

			if (!active)
				xingState->signalState = SIGNAL_STATE_MAIN_GATE_UP_SETUP;
			else if (0 == xingState->stateTimer)
				xingState->signalState = SIGNAL_STATE_4Q_GATE_DOWN_SETUP;
			break;

		case SIGNAL_STATE_4Q_GATE_DOWN_SETUP:
            xingState->lightsActive = true;
            xingState->bellActive = true;
            xingState->mainGatesActive = true;

            if (!active)
				xingState->signalState = SIGNAL_STATE_MAIN_GATE_UP_SETUP;
			else
			{
				xingState->stateTimer = DECISECS_SIGNAL_4Q_GATE_DROP;
				xingState->signalState = SIGNAL_STATE_4Q_GATE_DOWN;
			}
			break;
			
		case SIGNAL_STATE_4Q_GATE_DOWN:
            xingState->lightsActive = true;
            xingState->bellActive = true;
            xingState->mainGatesActive = true;
            xingState->auxGatesActive = true;

            if (!active)
				xingState->signalState = SIGNAL_STATE_4Q_GATE_UP_SETUP;
			else if (0 == xingState->stateTimer)
			{
				xingState->signalState = SIGNAL_STATE_ACTIVE_4Q;
			}
			break;
			
		case SIGNAL_STATE_ACTIVE_2Q:
            xingState->mainGatesActive = true;
            xingState->auxGatesActive = false;
            xingState->lightsActive = true;
            xingState->bellActive = (xingState->bellOnGateDropOnly)?false:true;

			if (!active)
				xingState->signalState = SIGNAL_STATE_MAIN_GATE_UP_SETUP;
			break;

		case SIGNAL_STATE_ACTIVE_4Q:
            xingState->lightsActive = true;
            xingState->bellActive = (xingState->bellOnGateDropOnly)?false:true;
            xingState->mainGatesActive = true;
            xingState->auxGatesActive = true;

			if (!active)
				xingState->signalState = SIGNAL_STATE_4Q_GATE_UP_SETUP;
				
			break;
		
			
		case SIGNAL_STATE_ACTIVE:
            xingState->lightsActive = true;
            xingState->bellActive = (xingState->bellOnGateDropOnly)?false:true;
            xingState->mainGatesActive = false;
            xingState->auxGatesActive = false;
			if (!active)
				xingState->signalState = SIGNAL_STATE_IDLE;
			break;

		case SIGNAL_STATE_4Q_GATE_UP_SETUP:
			xingState->auxGatesActive = false;
			
			if (xingState->gateConfig == GATES_4Q_DELAYED_UP)
			{
				xingState->stateTimer = DECISECS_SIGNAL_4Q_GATE_RISE_DELAY;
			} else {
				xingState->stateTimer = 0;
				xingState->mainGatesActive = false;
			}
			xingState->signalState = SIGNAL_STATE_4Q_GATE_UP;
			break;

		case SIGNAL_STATE_4Q_GATE_UP:
			xingState->auxGatesActive = false;
			if (active)
			{
				// Go back to active
				xingState->signalState = SIGNAL_STATE_MAIN_GATE_DOWN_SETUP;
			} else if (0 == xingState->stateTimer) {
				xingState->signalState = SIGNAL_STATE_MAIN_GATE_UP_SETUP;
			}
			break;

		case SIGNAL_STATE_MAIN_GATE_UP_SETUP:
			xingState->mainGatesActive = false;
			xingState->auxGatesActive = false;
			if (gatesEnabled)
			{
				xingState->stateTimer = DECISECS_SIGNAL_GATE_RISE;
			} else {
				xingState->stateTimer = 0;
			}
			xingState->signalState = SIGNAL_STATE_MAIN_GATE_UP;
			break;
			
		case SIGNAL_STATE_MAIN_GATE_UP:
			xingState->mainGatesActive = false;
			xingState->auxGatesActive = false;
			if (0 == xingState->stateTimer)
				xingState->signalState = SIGNAL_STATE_IDLE;
			break;
		
		default:
			xingState->signalState = SIGNAL_STATE_IDLE;
			break;
	}
}
