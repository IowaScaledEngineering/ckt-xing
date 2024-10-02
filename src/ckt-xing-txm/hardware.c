#include "hardware.h"

#define ADC_AVERAGING_CYCLES 8

void initializeOptions(DebounceState8_t* optionsDebouncer)
{
	initDebounceState8(optionsDebouncer, 0);
}

void readOptions(DebounceState8_t* optionsDebouncer)
{
	debounce8(0x1F & (~(PINA>>3)), optionsDebouncer);
}


uint8_t getTrackAInputs()
{
	uint8_t xingInputs = 0;
	
	if (!(PINA & _BV(PA0)))
			xingInputs |= INPUT_EAST_APPR_OCC;

	if (!(PINA & _BV(PA1)))
			xingInputs |= INPUT_ISLAND_OCC;

	if (!(PINA & _BV(PA2)))
			xingInputs |= INPUT_WEST_APPR_OCC;
			
			
	return xingInputs;
}

uint8_t getTrackBInputs()
{
	uint8_t xingInputs = 0;
	
	if (!(PINB & _BV(PB6)))
			xingInputs |= INPUT_EAST_APPR_OCC;

	if (!(PINB & _BV(PB5)))
			xingInputs |= INPUT_ISLAND_OCC;

	if (!(PINB & _BV(PB4)))
			xingInputs |= INPUT_WEST_APPR_OCC;
			
			
	return xingInputs;
}

void setCrossingActiveOutput(bool active)
{
	if (active)
		PORTB |= _BV(PB3);
	else
		PORTB &= ~_BV(PB3);
}

IndicatorLightState_t doBlinky(uint8_t ledPhase, IndicatorLightState_t ledA)
{
	if (ledA == LIGHT_SLOW_BLINK)
	{
		if (ledPhase >=10)
			ledA = LIGHT_ON;
		else
			ledA = LIGHT_OFF;
	}
	else if (ledA == LIGHT_FAST_BLINK)
	{
		if (ledPhase & 0x02)
			ledA = LIGHT_ON;
		else
			ledA = LIGHT_OFF;
	}
	else if (ledA == LIGHT_DOUBLE_SLOW_BLINK)
	{
		if (ledPhase >= 16)
			ledA = LIGHT_ON;
		else if (ledPhase >= 12)
			ledA = LIGHT_OFF;
		else if (ledPhase >= 8)
			ledA = LIGHT_ON;
		else
			ledA = LIGHT_OFF;
	}
	return ledA;
}

void setStatusLEDs(uint8_t ledPhase, IndicatorLightState_t ledA, IndicatorLightState_t ledB)
{
	ledA = doBlinky(ledPhase, ledA);
	ledB = doBlinky(ledPhase, ledB);

	//  PB1 - Output - Track A Active LED
	//  PB0 - Output - Track B Active LED

	if (LIGHT_OFF == ledA)
		PORTB &= ~_BV(PB1);
	else if (LIGHT_ON == ledA)
		PORTB |= _BV(PB1);

	if (LIGHT_OFF == ledB)
		PORTB &= ~_BV(PB0);
	else if (LIGHT_ON == ledB)
		PORTB |= _BV(PB0);
}



