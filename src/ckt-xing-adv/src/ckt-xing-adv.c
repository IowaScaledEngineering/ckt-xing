/*************************************************************************
Title:    CKT-XING-ADV
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
          Audio portions based on the work of David Johnson-Davies - www.technoblogy.com - 23rd October 2017
           and used under his Creative Commons Attribution 4.0 International license
File:     $Id: $
License:  GNU General Public License v3

CREDIT:
    The basic idea behind this playback design came from David Johson-Davies, who
    provided the basic framework and the place where I started.

LICENSE:
    Copyright (C) 2026 Michael Petersen, Nathan Holmes, with portions from 
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
#include <avr/sleep.h>
#include <stdbool.h>
#include <stdint.h>
#include "spiflash.h"
#include "audioAndLights.h"
#include "ispl.h"
#include "debouncer.h"
#include "systemHardware.h"
#include "crossingTrack.h"
#include "xingSignals.h"
#include "configuration.h"

SystemHardwareState_t state;
DebounceState8_t switchDebouncer;


void configSettingToLamps(SystemHardwareState_t* state, ConfigSetting_t configSetting, uint8_t val)
{
	for(uint8_t i=0; i<8; i++)
		state->configModeLamp[i]=false;
	state->configValueLamp = 0;

	switch(configSetting)
	{
		case CONFIG_SETTING_NONE:
			break;

		case CONFIG_SETTING_BELL_TYPE:
		case CONFIG_SETTING_APPROACH_TIMEOUT:
		case CONFIG_SETTING_ISLAND_TIMEOUT:
		case CONFIG_SETTING_LED_VS_LAMP:
		case CONFIG_SETTING_BELL_MODE:
		case CONFIG_SETTING_GATE_MODE:
			state->configModeLamp[configSetting - CONFIG_SETTING_BELL_TYPE] = true;
			state->configValueLamp = 0x0f & val;
			break;

		case CONFIG_SETTING_SERVO_UP_MAIN_1:
		case CONFIG_SETTING_SERVO_UP_MAIN_2:
		case CONFIG_SETTING_SERVO_UP_AUX_1:
		case CONFIG_SETTING_SERVO_UP_AUX_2:
			state->configModeLamp[6] = true;
			state->configValueLamp = 0x08>>(configSetting-CONFIG_SETTING_SERVO_UP_MAIN_1);					
			break;

		case CONFIG_SETTING_SERVO_DOWN_MAIN_1:
		case CONFIG_SETTING_SERVO_DOWN_MAIN_2:
		case CONFIG_SETTING_SERVO_DOWN_AUX_1:
		case CONFIG_SETTING_SERVO_DOWN_AUX_2:
			state->configModeLamp[7] = true;
			state->configValueLamp = 0x08>>(configSetting-CONFIG_SETTING_SERVO_DOWN_MAIN_1);
			break;

		default:
			for(uint8_t i=0; i<8; i++)
				state->configModeLamp[i]=true;
			break;
	}
}

uint8_t getLongPress(uint8_t switchState)
{
	static uint8_t longPressCounter[3] = {0,0,0};
	uint8_t retval = 0;
	uint8_t switchMask = 1;

	for(uint8_t i=0; i<3; i++, switchMask<<=1)
	{
		if (switchState & switchMask)
		{
			if (longPressCounter[i] < 0xFF)
				longPressCounter[i]++;
		}
		else
			longPressCounter[i] = 0;

		if (longPressCounter[i] > (SWITCH_LONG_PRESS_THRESHOLD_MS / SWITCH_UPDATE_TIME_MS))
			retval |= switchMask;
	}
	return retval;
}

void configureEverything(Configuration_t* config, CrossingTrack_t* trackA, CrossingTrack_t* trackB, CrossingSignalState_t* xingState, AudioAssetRecord* r)
{
	initCrossingSignals(xingState, config->configValues[CONFIG_SETTING_GATE_MODE], config->configValues[CONFIG_SETTING_BELL_MODE]==BELL_MODE_ONLY_ON_GATE_DROP);
	uint8_t approachTimeout = getApproachTimeoutDecisecs(config);
	uint8_t islandTimeout = getIslandTimeoutDecisecs(config);
	initializeCrossingTrack(trackA, approachTimeout, islandTimeout);
	initializeCrossingTrack(trackB, approachTimeout, islandTimeout);
	lightConfigSet(config->configValues[CONFIG_SETTING_LED_VS_LAMP] == LED_VS_LAMP_LED_MODE);
	isplAudioAssetLoad(config->configValues[CONFIG_SETTING_BELL_TYPE]-1, r);
}


int main(void)
{
	uint32_t lastUpdateTime = 0xf0000000;
	uint32_t lastSwitchTime = 0xf0000000;
	uint8_t configSetting = 0;

	Configuration_t globalConfig;
	SystemHardwareState_t state;
	MenuState_t menuState = MENU_OFF;
	CrossingTrack_t trackA;
	CrossingTrack_t trackB;
	CrossingSignalState_t xingState;
	AudioAssetRecord r;
	uint8_t i=0;
	
	// Deal with watchdog first thing
	MCUSR = 0;              // Clear reset status
	wdt_reset();            // Reset the WDT, just in case it's still enabled over reset
	wdt_enable(WDTO_1S);    // Enable it at a 1S timeout.

	// PORT A
	//  PA7 - Output - /SHUTDOWN to amplifier
	//  PA6 - Input  - Switch UP (enable pullup)
	//  PA5 - Output - SCL
	//  PA4 - I/O    - SDA
	//  PA3 - Output - /CS to flash
	//  PA2 - Output - CLK to flash
	//  PA1 - Output - MOSI to flash
	//  PA0 - Input  - MISO to flash (enable pullup)

	// PORT B
	//  PB7 - n/a    - /RESET (not I/O pin)
	//  PB6 - Input  - /EXT_IN
	//  PB5 - Input  - Switch DOWN (enable pullup)
	//  PB4 - Input  - Switch NEXT (enable pullup)
	//  PB3 - Output - Audio PWM Output
	//  PB2 - Output - Center Light
	//  PB1 - Output - Right Light
	//  PB0 - Output - Left Light


	// DDR 1=output, 0=input
	PORTA = 0b01011001;
	DDRA  = 0b10101110;

	PORTB = 0b11110000; 
	DDRB  = 0b00001111;

	audioInitialize();
	spiSetup();
	spiflashReset();

	sei();
	wdt_reset();

	isplInitialize();
	initDebounceState8(&switchDebouncer, getSwitches());
	loadConfigValues(&globalConfig);
	configureEverything(&globalConfig, &trackA, &trackB, &xingState, &r);
	initSystemHardwareState(&state, &globalConfig);

	while(1)
	{
		wdt_reset();
		audioPump();

		uint32_t currentTime = getMillis();

		// Handle Switches and Menuing
		if ((uint32_t)(currentTime - lastSwitchTime) > SWITCH_UPDATE_TIME_MS)
		{
			static uint8_t increments = 0;
			lastSwitchTime = currentTime;
			uint8_t switchesPressed = debounce8(getSwitches(), &switchDebouncer);
			// Invert the three switch states, since they're active low
			uint8_t switchState = 0x07 ^ getDebouncedState(&switchDebouncer); 
			// Feed switch states into the long press detector
			uint8_t longPress = getLongPress(switchState);

			switch(menuState)
			{
				case MENU_RESET_START:
					state.configModeLamp[7]=true;	
					for(i=0; i<7; i++)
						state.configModeLamp[i]=false;

					if (switchState == 0)
					{
						increments = 0;
						menuState = MENU_RESET_CONFIRM;
					}
					break;

				case MENU_RESET_CONFIRM:
					if (longPress == SWITCH_NEXT_MASK)
					{
						increments = 0;
						menuState = MENU_SAVE;
						break;
					}

					if (switchState == SWITCH_UP_MASK)
					{
						increments++;
				
						if (increments > 32)
						{
							factoryInitConfiguration(&globalConfig);
							while(1); // Loop and let the WDT get us
						} else {
							if (increments % 8 == 7)
								state.configValueLamp = (0x01 | (state.configValueLamp<<1));
						}

					} else {
						increments = 0;
						state.configValueLamp = 0;
					}

					break;

				case MENU_DIAGNOSTIC_START:
					increments = 0;
					for(i=0; i<8; i++)
						state.configModeLamp[i] = false;
					xingState.lightsActive = true;

				case MENU_DIAGNOSTIC:
					increments++;
					menuState = MENU_DIAGNOSTIC;
					trackA.active = (PINB & _BV(PB6));
					trackA.ledState = trackA.active?LIGHT_ON:LIGHT_OFF;
					trackB.ledState = trackA.active?LIGHT_OFF:LIGHT_ON;
					state.configValueLamp = (trackA.active)?0x0A:0x05;
					// Turn off the mode lamp 

					for(i=0; i<8; i++)
						state.configModeLamp[i] = (1<<i) & getDebouncedState(&state.trackSensorDebouncer);

					xingState.bellActive = switchState & SWITCH_DOWN_MASK;

					if (trackA.active)
					{
						xingState.mainGatesActive = true;
						xingState.auxGatesActive = false;
					} else {
						xingState.mainGatesActive = false;
						xingState.auxGatesActive = true;
					}
					


					break;

				case MENU_OFF:
					// Configuration is current off, system is running normally
					if ((SWITCH_NEXT_MASK & longPress) && (SWITCH_DOWN_MASK & switchState))
						menuState = MENU_RESET_START;
					else if ((SWITCH_NEXT_MASK & longPress) && (SWITCH_UP_MASK & switchState))
						menuState = MENU_DIAGNOSTIC_START;
					else if (SWITCH_NEXT_MASK == longPress)
						menuState = MENU_START;
					break;

				case MENU_START:
					state.configModeLamp[0]=true;	
					for(i=1; i<8; i++)
						state.configModeLamp[i]=false;

					if (switchState != SWITCH_NEXT_MASK)
					{
						// load initial configuration value
						configSetting = CONFIG_SETTING_BELL_TYPE;
						
						// Ready to run
						menuState = MENU_RUN;
					}
					break;

				case MENU_RUN:
					// First check to see if we're waiting on a long press to save and exit
					if (longPress == SWITCH_NEXT_MASK)
					{
						menuState = MENU_SAVE;
						break;
					}

					if (switchesPressed == SWITCH_NEXT_MASK)
					{
						// If next is pressed, advance to next setting
						if (++configSetting >= CONFIG_SETTING_NONE)
							configSetting = CONFIG_SETTING_BELL_TYPE;
					} 
					else if (switchesPressed == SWITCH_UP_MASK) 
					{
						if (configSetting < CONFIG_SETTING_SERVO_UP_MAIN_1)
						{
							if (globalConfig.configValues[configSetting] < globalConfig.configValueMaximums[configSetting])
								globalConfig.configValues[configSetting]++;
						}
						else if (configSetting < CONFIG_SETTING_NONE) 
						{
							// Do Servo Upper Limit
							globalConfig.servoLimit[configSetting - CONFIG_SETTING_SERVO_UP_MAIN_1] = globalConfig.servoLimit[configSetting - CONFIG_SETTING_SERVO_UP_MAIN_1] + (10<<4);
						}
					}
					else if (switchesPressed == SWITCH_DOWN_MASK) 
					{
						if (configSetting < CONFIG_SETTING_SERVO_UP_MAIN_1)
						{
							if (globalConfig.configValues[configSetting] > 1)
								globalConfig.configValues[configSetting]--;
						}
						else if (configSetting < CONFIG_SETTING_NONE) 
						{
							// Do Servo Upper Limit
							globalConfig.servoLimit[configSetting - CONFIG_SETTING_SERVO_UP_MAIN_1] = globalConfig.servoLimit[configSetting - CONFIG_SETTING_SERVO_UP_MAIN_1] - (10<<4);
						}
					} 
					
					if (configSetting >= CONFIG_SETTING_SERVO_UP_MAIN_1 && configSetting <= CONFIG_SETTING_SERVO_DOWN_AUX_2)
					{
						uint8_t servoNum = (configSetting - CONFIG_SETTING_SERVO_UP_MAIN_1);
						state.servoPosition[servoNum%4] = globalConfig.servoLimit[servoNum];
					}
					configSettingToLamps(&state, configSetting, globalConfig.configValues[configSetting]);
					break;

				case MENU_SAVE:
					configSetting = CONFIG_SETTING_NONE;  // 0 is essentially "run normally"

					saveConfigValues(&globalConfig);
					configureEverything(&globalConfig, &trackA, &trackB, &xingState, &r);
					configSettingToLamps(&state, configSetting, 0);
					if (switchState != SWITCH_NEXT_MASK)
						menuState = MENU_OFF;
					break;

				case MENU_END:
				default:
					configSetting = CONFIG_SETTING_NONE;  // 0 is essentially "run normally"
					configSettingToLamps(&state, configSetting, 0);
					menuState = MENU_OFF;
					break;
			}
		}

		audioPump();

		if (((uint32_t)currentTime - lastUpdateTime) > LOOP_UPDATE_TIME_MS)
		{
			static uint8_t ticks = 0;
			lastUpdateTime = currentTime;

			// Read inputs should run every 50mS
			readInputs(&state);

			// Each run through the loop is 50mS apart, so a decisec is 2 runs
			ticks = (ticks+1) % 2;

			// The state machines should only run every 100mS, because all of the timeouts
			//  internally are based around deciseconds.  Running the read routine 4x
			//  faster allows a fresh set of debounced inputs every time we run the state machines
			//  Also, they should only be run if we're not doing menu stuff

			if (MENU_OFF == menuState)
			{
				runCrossingTrackStateMachine(&trackA, getTrackAState(&state),  0 == ticks);

				runCrossingTrackStateMachine(&trackB, getTrackBState(&state),  0 == ticks);

				bool activeCrossing = isCrossingTrackActive(&trackA) || isCrossingTrackActive(&trackB) || isExtInActive(&state);

				runCrossingSignalStateMachine(&xingState, activeCrossing, 0 == ticks);
			}

			bool activateOutputs = (MENU_OFF == menuState || MENU_DIAGNOSTIC == menuState);

			if(xingState.lightsActive && activateOutputs )
				activateLights();
			else
				deactivateLights();

			audioPump();

			if(xingState.bellActive && activateOutputs)
			{
				if(!audioIsPlaying())
					audioPlay(r.addr, r.size, r.sampleRate, true);
			}
			else
				stopAudioRepeat();

			state.mainGatesActive = xingState.mainGatesActive;
			state.auxGatesActive = xingState.auxGatesActive;
			state.trkStatusA = trackA.ledState;
			state.trkStatusB = trackB.ledState;
			// Run outputs every time.  
			setOutputs(&state, &globalConfig, activateOutputs);
		}
	}
}




