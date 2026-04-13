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

#ifndef _CONFIGURATION_H_
#define _CONFIGURATION_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum 
{
	MENU_OFF                 = 0,
	MENU_START,
	MENU_RUN,
	MENU_SAVE,
	MENU_RESET_START,
	MENU_RESET_CONFIRM,
	MENU_DIAGNOSTIC_START,
	MENU_DIAGNOSTIC,
	MENU_END
} MenuState_t;

typedef enum
{
	CONFIG_SETTING_BELL_TYPE          = 0,
	CONFIG_SETTING_APPROACH_TIMEOUT   = 1,
	CONFIG_SETTING_ISLAND_TIMEOUT     = 2,
	CONFIG_SETTING_LED_VS_LAMP        = 3,
	CONFIG_SETTING_BELL_MODE          = 4,
	CONFIG_SETTING_GATE_MODE          = 5,
	CONFIG_SETTING_SERVO_UP_MAIN_1    = 6,
	CONFIG_SETTING_SERVO_UP_MAIN_2    = 7,
	CONFIG_SETTING_SERVO_UP_AUX_1     = 8,
	CONFIG_SETTING_SERVO_UP_AUX_2     = 9,
	CONFIG_SETTING_SERVO_DOWN_MAIN_1  = 10,
	CONFIG_SETTING_SERVO_DOWN_MAIN_2  = 11,
	CONFIG_SETTING_SERVO_DOWN_AUX_1   = 12,
	CONFIG_SETTING_SERVO_DOWN_AUX_2   = 13,
	CONFIG_SETTING_NONE,
	CONFIG_SETTING_END
} ConfigSetting_t;

#define BELL_MODE_ALWAYS_ON          1
#define BELL_MODE_ONLY_ON_GATE_DROP  2

#define LED_VS_LAMP_LAMP_MODE        1
#define LED_VS_LAMP_LED_MODE         2

#define SERVO_MAIN_1_LIMIT_UP   0
#define SERVO_MAIN_2_LIMIT_UP   1
#define SERVO_AUX_1_LIMIT_UP    2
#define SERVO_AUX_2_LIMIT_UP    3
#define SERVO_MAIN_1_LIMIT_DOWN 4
#define SERVO_MAIN_2_LIMIT_DOWN 5
#define SERVO_AUX_1_LIMIT_DOWN  6
#define SERVO_AUX_2_LIMIT_DOWN  7


typedef struct 
{
	uint8_t configValues[6];
	uint8_t configValueMaximums[6];
	uint16_t servoLimit[8];// = {SERVO_180_DEGREES<<4, SERVO_180_DEGREES<<4, SERVO_180_DEGREES<<4, SERVO_180_DEGREES<<4};
    int16_t servoRate[4];// = {SERVO_STEP, SERVO_STEP, SERVO_STEP, SERVO_STEP};
} Configuration_t;

void loadConfigValues(Configuration_t* config);
void factoryInitConfiguration(Configuration_t* config);
void saveConfigValues(Configuration_t* config);
uint8_t getIslandTimeoutDecisecs(Configuration_t* config);
uint16_t getApproachTimeoutDecisecs(Configuration_t* config);

#endif