/*************************************************************************
Title:    MSS-CASCADE-BASIC
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     optionSwitches.h
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

#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#include "debouncer.h"
#include "crossingTrack.h"
#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>

uint8_t getTrackAInputs();
uint8_t getTrackBInputs();
void setCrossingActiveOutput(bool active);
void setStatusLEDs(uint8_t ledPhase, IndicatorLightState_t ledA, IndicatorLightState_t ledB);
void initializeOptions(DebounceState8_t* optionsDebouncer);
void readOptions(DebounceState8_t* optionsDebouncer);

#endif
