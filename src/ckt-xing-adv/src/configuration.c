#include <avr/eeprom.h>
#include "configuration.h"
#include "xingSignals.h"
#include "systemHardware.h"

#define EEPROM_BELL_TYPE           0x10
#define EEPROM_APPROACH_TIMEOUT    0x11
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



void setServoRates(Configuration_t* config)
{
    // Calculate servo rates
	for (uint8_t i=0; i<4; i++)
	{
        config->servoRate[i] = (int16_t)(config->servoLimit[i] - config->servoLimit[i+4])/((DECISECS_SIGNAL_MAIN_GATE_DROP*100) / LOOP_UPDATE_TIME_MS);
	}
}

void saveConfigValues(Configuration_t* config)
{
	for(uint8_t i=CONFIG_SETTING_BELL_TYPE; i<=CONFIG_SETTING_GATE_MODE; i++)
	{
		eeprom_write_byte((uint8_t*)EEPROM_BELL_TYPE + i, config->configValues[i]);
	}

	for (uint8_t i=0; i<8; i++)
	{
		eeprom_write_word((uint16_t*)(EEPROM_SERVO_0_UP + i*2), config->servoLimit[i]);
	}
    setServoRates(config);
}

void factoryInitConfiguration(Configuration_t* config)
{
	// FIXME - factory reset here
	config->configValues[CONFIG_SETTING_BELL_TYPE] = 1; 
	config->configValues[CONFIG_SETTING_APPROACH_TIMEOUT] = 5; 
	config->configValues[CONFIG_SETTING_ISLAND_TIMEOUT] = 4; 
	config->configValues[CONFIG_SETTING_LED_VS_LAMP] = 1; 
	config->configValues[CONFIG_SETTING_BELL_MODE] = 1; 
	config->configValues[CONFIG_SETTING_GATE_MODE] = GATES_2Q_ONLY; 
	for (uint8_t i=0; i<8; i++)
	{
		config->servoLimit[i] = (i<4)?SERVO_180_DEGREES<<4:SERVO_0_DEGREES<<4;
	}
	saveConfigValues(config);
}

void loadConfigValues(Configuration_t* config)
{
	config->configValueMaximums[CONFIG_SETTING_BELL_TYPE] = 7;
	config->configValueMaximums[CONFIG_SETTING_APPROACH_TIMEOUT] = 15;
	config->configValueMaximums[CONFIG_SETTING_ISLAND_TIMEOUT] = 15;
	config->configValueMaximums[CONFIG_SETTING_LED_VS_LAMP] = 2;
	config->configValueMaximums[CONFIG_SETTING_BELL_MODE] = 2;
	config->configValueMaximums[CONFIG_SETTING_GATE_MODE] = GATES_END-1;

	uint8_t i = eeprom_read_byte((const uint8_t*)EEPROM_BELL_TYPE);
	if (i<1 || i>15)
	{
		// It's probably crap, factory reset
		factoryInitConfiguration(config);
	}

	for(uint8_t i=CONFIG_SETTING_BELL_TYPE; i<=CONFIG_SETTING_GATE_MODE; i++)
		config->configValues[i] = eeprom_read_byte((const uint8_t*)EEPROM_BELL_TYPE + (i));

	for (uint8_t i=0; i<8; i++)
	{
		config->servoLimit[i] = eeprom_read_word((const uint16_t*)(EEPROM_SERVO_0_UP + i*2));
	}
    setServoRates(config);
}

uint16_t getApproachTimeoutDecisecs(Configuration_t* config)
{
	const uint16_t approachTimeouts[15] = {50, 75, 100, 125, 150, 175, 200, 225, 250, 275, 300, 325, 350, 375, 400};
	uint8_t index = MIN(sizeof(approachTimeouts)/sizeof(approachTimeouts[0]), config->configValues[CONFIG_SETTING_APPROACH_TIMEOUT]-1);
	return approachTimeouts[index];
}

uint8_t getIslandTimeoutDecisecs(Configuration_t* config)
{
	const uint8_t approachTimeouts[15] = {5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 60, 70, 80, 100, 150};
	uint8_t index = MIN(sizeof(approachTimeouts)/sizeof(approachTimeouts[0]), config->configValues[CONFIG_SETTING_APPROACH_TIMEOUT]-1);
	return approachTimeouts[index];
}
