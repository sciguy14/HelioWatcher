/*! \file i2cswconf.h \brief Software-driven I2C interface configuration. */
//*****************************************************************************
//
// File Name	: 'i2cswconf.h'
// Title		: software-driven I2C interface using port pins
// Author		: Pascal Stang - Copyright (C) 2000-2002
// Created		: 11/22/2000
// Revised		: 5/2/2002
// Version		: 1.1
// Target MCU	: Atmel AVR series
// Editor Tabs	: 4
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************

#ifndef I2CSWCONF_H
#define I2CSWCONF_H

// clock line port
#define SCLPORT	PORTJ	// i2c clock port
#define SCLDDR	DDRJ	// i2c clock port direction
// data line port
#define SDA_A_PORT	PORTA	// i2c data port
#define SDA_A_DDR	DDRA	// i2c data port direction
#define SDA_A_PIN	PINA	// i2c data port input
#define SDA_B_PORT	PORTJ	// i2c data port
#define SDA_B_DDR	DDRJ	// i2c data port direction
#define SDA_B_PIN	PINJ	// i2c data port input
// pin assignments
#define SCL		PJ5		// i2c clock pin
#define SDA_A		PA5		// i2c data pin
#define SDA_B		PJ6		// i2c data pin

#endif
