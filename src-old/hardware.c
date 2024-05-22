#include <stdlib.h>
#include <avr/io.h>
#include <stdbool.h>

void init_gpio()
{
	// PORTB Bit Assignment
	// PORTB:7 - Main Gates
	// PORTB:6 - 4Q Secondary Gates
	// PORTB:5 - SCK (programming only)
	// PORTB:4 - MISO (programming only)
	// PORTB:3 - MOSI (programming only)
	// PORTB:2 - Left Lamps
	// PORTB:1 - Right Lamps
	// PORTB:0 - Const Lamps
	PORTB = 0x00;
	DDRB = 0xFF;
	
	// PORTC Bit Assignment
	// PORTC:7 - n/a
	// PORTC:6 - /RESET pin, not used for I/O
	// PORTC:5 - SW4 input (negative logic)
	// PORTC:4 - Island Occupancy (positive logic)
	// PORTC:3 - East Approach Occupancy (positive logic)
	// PORTC:2 - West Approach Occupancy (positive logic)
	// PORTC:1 - Active In (positive logic)
	// PORTC:0 - Active Out (positive logic)
	PORTC = 0xFE;  //  All pullups on
	DDRC = 0x01;

	// PORTD Bit Assignment
	// PORTD:7 - SW3 input (negative logic)
	// PORTD:6 - SW2 input (negative logic)
	// PORTD:5 - SW1 input (negative logic)
	// PORTD:4 - Bell enable (negative logic)
	// PORTD:3 - SW8 input (negative logic)
	// PORTD:2 - SW7 input (negative logic)
	// PORTD:1 - SW6 input (negative logic)
	// PORTD:0 - SW5 input (negative logic)
	PORTD = 0xEF;  //  All pullups on
	DDRD = 0x10;
}


void activateRightLamp()
{
	PORTB |= _BV(PB1);
}

void deactivateRightLamp()
{
	PORTB &= ~_BV(PB1);
}

void activateLeftLamp()
{
	PORTB |= _BV(PB0);
}

void deactivateLeftLamp()
{
	PORTB &= ~_BV(PB0);
}

void activateConstLamp()
{
	PORTB |= _BV(PB2);
}

void deactivateConstLamp()
{
	PORTB &= ~_BV(PB2);
}

uint8_t getSwitchState()
{
	uint8_t retval = 0;
	retval = (PIND>>5) & 0x07;
	retval |= (PIND<< 4) & 0xF0;
	retval |= (PINC & 0x20)?0x08:0x00; // Get SW4 from Port C
	return (~retval); // Invert since these are all negative logic (switch on = low)
}

uint8_t getInputState()
{
	uint8_t retval = 0;
	retval = (PINC>>1) & 0x0F;
	return retval;
}

void activateMainGates()
{
	PORTB |= _BV(PB6);
}

void deactivateMainGates()
{
	PORTB &= ~_BV(PB6);
}

void activateSecondaryGates()
{
	PORTB |= _BV(PB7);
}

void deactivateSecondaryGates()
{
	PORTB &= ~_BV(PB7);
}

void setDetectionActive()
{
	PORTC |= _BV(PC0);
}

void clearDetectionActive()
{
	PORTC &= ~_BV(PC0);
}

void activateBell()
{
	PORTD &= ~_BV(PD4);
}

void deactivateBell()
{
	PORTD |= _BV(PD4);
}
