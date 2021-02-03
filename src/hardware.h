#include <stdlib.h>
#include <avr/io.h>
#include <stdbool.h>

void init_gpio();

void activateMainGates();
void deactivateMainGates();

void activateSecondaryGates();
void deactivateSecondaryGates();

void setCrossingActive();
void clearCrossingActive();

void activateBell();
void deactivateBell();

uint8_t getSwitchState();
uint8_t getInputState();



#define INPUT_XING_ACTIVE     0x01
#define INPUT_WEST_APPR_OCC   0x02
#define INPUT_EAST_APPR_OCC   0x04
#define INPUT_ISLAND_OCC      0x08

