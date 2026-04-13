/*************************************************************************
Title:    Big-Bang I2C Library
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     i2c.h
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

#include "i2c.h"

static inline void sda_low() { I2C_DDR |= _BV(SDA); I2C_PORT &= ~_BV(SDA); _delay_us(I2C_DELAY_US); }
static inline void sda_high() { I2C_DDR &= ~_BV(SDA); I2C_PORT |= _BV(SDA); _delay_us(I2C_DELAY_US); }
static inline void scl_low() { I2C_PORT &= ~_BV(SCL); _delay_us(I2C_DELAY_US); }
static inline void scl_high() { I2C_PORT |= _BV(SCL); _delay_us(I2C_DELAY_US); }


void i2cStart(void)
{
	scl_high();
	sda_low();
	scl_low();
	sda_high();
}

void i2cStop(void)
{
	scl_low();
	sda_low();
	scl_high();
	sda_high();
}

bool i2cWriteByte(uint8_t byte)
{
	uint8_t i = 0x80, ack;

	do
	{
		if(byte & i)
		{
			sda_high();
		}
		else
		{
			sda_low();
		}
		
		scl_high();
		scl_low();
		
		i >>= 1;
	} while(i);

	sda_high();  // Release SDA
	
	scl_high();
	if(I2C_PIN & _BV(SDA))
		ack = 0;
	else
		ack = 1;
	scl_low();

	return (bool)ack;
}

uint8_t i2cReadByte(uint8_t ack)
{
	uint8_t i=0, data = 0;

	for(i=0; i<8; i++)
	{
		data<<=1;
		scl_high();
		if (I2C_PIN & _BV(SDA))
			data |= 0x01;
		scl_low();
	}
	
	if(ack)
		sda_low();
	scl_high();
	scl_low();
	sda_high();

	return data;
}

bool writeByte(uint8_t addr, uint8_t cmd, uint8_t writeVal)
{
	return writeBytes(addr, cmd, &writeVal, 1);
}

bool writeBytes(uint8_t addr, uint8_t cmd, uint8_t* writeVal, uint8_t writeValLen)
{
	bool ack = true;
	uint8_t i=0;	
	i2cStart();
	
	i2cWriteByte(addr << 1);
	i2cWriteByte(cmd);
	for(i=0; i<writeValLen; i++)
	{
		ack = i2cWriteByte(*(writeVal+i));
		if (!ack)
			break;
	}

	i2cStop();

	return ack;
}

bool readByte(uint8_t addr, uint8_t cmd, uint8_t* data)
{
	bool ack = true;
	*data = 0x00;
	
	i2cStart();
	
	ack &= i2cWriteByte(addr << 1);
	ack &= i2cWriteByte(cmd);

	i2cStart();

	ack &= i2cWriteByte((addr << 1) | 0x01);
	*data = i2cReadByte(0);
	i2cStop();
	return ack;
}


