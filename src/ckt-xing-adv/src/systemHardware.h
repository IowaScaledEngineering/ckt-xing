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

#ifndef _SYSTEM_HARDWARE_H_
#define _SYSTEM_HARDWARE_H_

#include <stdint.h>
#include <stdbool.h>
#include "debouncer.h"
#include "i2c.h"
#include "timings.h"
#include "configuration.h"

#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))

// Timings

#define LOOP_UPDATE_TIME_MS       50
#define SWITCH_UPDATE_TIME_MS     25
#define SWITCH_LONG_PRESS_THRESHOLD_MS 2000UL

// PORT A
//  PA7 - Output - /SHUTDOWN to amplifier
//  PA6 - Input  - Switch UP
//  PA5 - Output - SCL
//  PA4 - I/O    - SDA
//  PA3 - Output - /CS to flash
//  PA2 - Output - CLK to flash
//  PA1 - Output - MOSI to flash
//  PA0 - Input  - MISO to flash (enable pullup)

// PORT B
//  PB7 - n/a    - /RESET (not I/O pin)
//  PB6 - Input  - /EXT_IN
//  PB5 - Input  - Switch DOWN
//  PB4 - Input  - Switch NEXT
//  PB3 - Output - Audio PWM Output
//  PB2 - Output - Center Light
//  PB1 - Output - Right Light
//  PB0 - Output - Left Light

#define SWITCH_UP          PA6
#define SWITCH_UP_PIN      PINA
#define SWITCH_DOWN        PB5
#define SWITCH_DOWN_PIN    PINB
#define SWITCH_NEXT        PB4
#define SWITCH_NEXT_PIN    PINB

#define SWITCH_UP_MASK     0x04
#define SWITCH_DOWN_MASK   0x02
#define SWITCH_NEXT_MASK   0x01


#define CONFIG_LEDS        8

#define AUDIO_AMP_EN       PA7

#define I2C_PORT           PORTA
#define I2C_PIN            PINA
#define I2C_DDR            DDRA
#define I2C_DELAY_US       2

#define SDA                PA4
#define SCL                PA5


#define EXT_IN_PIN         PINB
#define EXT_IN             PB6         
#define EXT_IN_ACTIVE_MASK 0x01


#define LAMP_PORT          PORTB
#define LAMP_CENTER        PB2
#define LAMP_RIGHT         PB1
#define LAMP_LEFT          PB0

/*#define CONFIG_BELL_TYPE      0
#define CONFIG_APRCH_TIMEOUT  1
#define CONFIG_ISLAND_TIMEOUT 2
#define CONFIG_LED_OR_LAMP    3
#define CONFIG_BELL_MODE      4
#define CONFIG_GATE_MODE      5
#define CONFIG_RESERVED       6
#define CONFIG_SERVO_1_UP     7
#define CONFIG_SERVO_2_UP     8
#define CONFIG_SERVO_3_UP     9
#define CONFIG_SERVO_4_UP    10
#define CONFIG_SERVO_1_DOWN  11
#define CONFIG_SERVO_2_DOWN  12
#define CONFIG_SERVO_3_DOWN  13
#define CONFIG_SERVO_4_DOWN  14*/


#define PCA9685_ADR_0         0x40
#define PCA9685_REG_MODE_1    0x00
#define PCA9685_REG_MODE_2    0x01

#define PCA9685_REG_CH0_ON_L  0x06
#define PCA9685_REG_CH0_ON_H  0x07
#define PCA9685_REG_CH0_OFF_L 0x08
#define PCA9685_REG_CH0_OFF_H 0x09

#define PCA9685_REG_CH1_ON_L  0x0A
#define PCA9685_REG_CH1_ON_H  0x0B
#define PCA9685_REG_CH1_OFF_L 0x0C
#define PCA9685_REG_CH1_OFF_H 0x0D

#define PCA9685_REG_CH2_ON_L  0x0E
#define PCA9685_REG_CH2_ON_H  0x0F
#define PCA9685_REG_CH2_OFF_L 0x10
#define PCA9685_REG_CH2_OFF_H 0x11

#define PCA9685_REG_CH3_ON_L  0x12
#define PCA9685_REG_CH3_ON_H  0x13
#define PCA9685_REG_CH3_OFF_L 0x14
#define PCA9685_REG_CH3_OFF_H 0x15

#define PCA9685_REG_CH6_ON_L  0x1E
#define PCA9685_REG_CH6_ON_H  0x1F

#define PCA9685_REG_ALL_ON_L  0xFA
#define PCA9685_REG_ALL_ON_H  0xFB
#define PCA9685_REG_ALL_OFF_L 0xFC
#define PCA9685_REG_ALL_OFF_H 0xFD
#define PCA9685_REG_PRESCALE  0xFE


#define PCA9685_VAL_50HZ_PRESCALE  0x7A

#define EEPROM_BELL_TYPE           0x10
#define EEPROM_APPROACHCH_TIMEOUT  0x11
#define EEPROM_ISLAND_TIMEOUT      0x12
#define EEPROM_LED_VS_LAMP         0x13
#define EEPROM_BELL_MODE           0x14
#define EEPROM_GATE_MODE           0x15
#define EEPROM_SERVO_0_UP          0x20
#define EEPROM_SERVO_1_UP          0x22
#define EEPROM_SERVO_2_UP          0x24
#define EEPROM_SERVO_3_UP          0x26
#define EEPROM_SERVO_0_DOWN        0x28
#define EEPROM_SERVO_1_DOWN        0x2A
#define EEPROM_SERVO_2_DOWN        0x2C
#define EEPROM_SERVO_3_DOWN        0x2E




#define TCA9555_ADDR_000  0x20
#define TCA9555_GPIN0        0
#define TCA9555_GPIN1        1
#define TCA9555_GPOUT0       2
#define TCA9555_GPOUT1       3
#define TCA9555_GPDDR0       6
#define TCA9555_GPDDR1       7

#define GPOUT1_MAIN_GATES_UP_MASK    0b01000000
#define GPOUT1_MAIN_GATES_DOWN_MASK  0b10000000
#define GPOUT1_AUX_GATES_UP_MASK     0b00010000
#define GPOUT1_AUX_GATES_DOWN_MASK   0b00100000

#define GPOUT1_CONFIG_VAL_A_LED_MASK 0b00000001
#define GPOUT1_CONFIG_VAL_B_LED_MASK 0b00000010
#define GPOUT1_CONFIG_VAL_C_LED_MASK 0b00000100
#define GPOUT1_CONFIG_VAL_D_LED_MASK 0b00001000

// Servo Position Stuff
#define SERVO_MAX_DEGREES  (515UL<<4)
#define SERVO_180_DEGREES  410UL
#define SERVO_0_DEGREES    205UL
#define SERVO_MIN_DEGREES  (100UL<<4)

#define SERVO_STEP         52 
// (((SERVO_180_DEGREES-SERVO_0_DEGREES)<<4)/60UL)

typedef enum{
	TRACK_STATUS_OFF = 0,
	TRACK_STATUS_ON,
	TRACK_STATUS_FAST_BLINK,
	TRACK_STATUS_SLOW_BLINK
} TrackStatusLED_t;


typedef struct
{
	bool configModeLamp[CONFIG_LEDS];
	uint8_t configValueLamp;
	TrackStatusLED_t trkStatusA;
	TrackStatusLED_t trkStatusB;
	bool mainGatesActive;
	bool auxGatesActive;
	DebounceState8_t trackSensorDebouncer;
    DebounceState8_t extInDebouncer;
	uint16_t servoPosition[4];// = {SERVO_180_DEGREES<<4, SERVO_180_DEGREES<<4, SERVO_180_DEGREES<<4, SERVO_180_DEGREES<<4};
} SystemHardwareState_t;


bool initPCA9685();
bool initTCA9555();
bool initSystemHardwareState(SystemHardwareState_t* state);
void readInputs(SystemHardwareState_t* state);
uint8_t getTrackAState(SystemHardwareState_t* state);
uint8_t getTrackBState(SystemHardwareState_t* state);
void setOutputs(SystemHardwareState_t* state, Configuration_t* config, bool driveGates);
bool isExtInActive(SystemHardwareState_t* state);
uint8_t getSwitches();
void writeServoPosition(uint8_t baseRegister, uint16_t servoPosition);
#endif
