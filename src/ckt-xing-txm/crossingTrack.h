/*************************************************************************
Title:    Crossing Track Detection State Machine 
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     crossingTrack.h
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

#ifndef _CROSSINGTRACK_H_
#define _CROSSINGTRACK_H_

#include <stdint.h>
#include <stdbool.h>
#include "debouncer.h"

#define DECISECS_DETECTION_MINIMUM_ON          20
#define DECISECS_DETECTION_APPROACH_TIMEOUT   200
#define DECISECS_DETECTION_LOCKOUT_TIMEOUT    200

#define INPUT_WEST_APPR_OCC   0x01
#define INPUT_EAST_APPR_OCC   0x02
#define INPUT_ISLAND_OCC      0x04

typedef enum
{
	DETECTION_STATE_IDLE = 0,
	DETECTION_STATE_ISLAND_SURPRISE,
	DETECTION_STATE_WEST_APPROACH_SETUP,
	DETECTION_STATE_WEST_APPROACH,
	DETECTION_STATE_WEST_ACTIVE,
	DETECTION_STATE_WEST_DET_LOCKOUT_SETUP,
	DETECTION_STATE_WEST_DET_LOCKOUT,
	DETECTION_STATE_WEST_TIME_LOCKOUT_SETUP,
	DETECTION_STATE_WEST_TIME_LOCKOUT,
	DETECTION_STATE_WEST_TIMEOUT,
	
	DETECTION_STATE_EAST_APPROACH_SETUP,
	DETECTION_STATE_EAST_APPROACH,
	DETECTION_STATE_EAST_ACTIVE,
	DETECTION_STATE_EAST_DET_LOCKOUT_SETUP,
	DETECTION_STATE_EAST_DET_LOCKOUT,
	DETECTION_STATE_EAST_TIME_LOCKOUT_SETUP,
	DETECTION_STATE_EAST_TIME_LOCKOUT,
	DETECTION_STATE_EAST_TIMEOUT,
} DetectionState_t;  

typedef enum
{
	LIGHT_OFF = 0,
	LIGHT_ON,
	LIGHT_SLOW_BLINK,
	LIGHT_FAST_BLINK,
	LIGHT_DOUBLE_SLOW_BLINK
} IndicatorLightState_t;  


typedef struct
{
	DebounceState8_t sensorDebouncer;
	DetectionState_t trackState;
	IndicatorLightState_t ledState;
	bool active;
	uint16_t stateTimer;
	uint8_t timeIslandTimeout;
	uint16_t timeApproachTimeout;
	uint8_t timeApproachReset;
	uint16_t timeDetectionLockout;
} CrossingTrack_t;

void initializeCrossingTrack(CrossingTrack_t* t);
bool isCrossingTrackActive(CrossingTrack_t* t);
void setCrossingMinimumOn(CrossingTrack_t* t, uint16_t decisecs);
void setCrossingApproachTimeout(CrossingTrack_t* t, uint16_t decisecs);
void setCrossingDetectionLockout(CrossingTrack_t* t, uint16_t decisecs);
void runCrossingTrackStateMachine(CrossingTrack_t* t, uint8_t xingInputs, bool isDecisecTick);
IndicatorLightState_t getStatusLEDState(CrossingTrack_t* t);

#endif

