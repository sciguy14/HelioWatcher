/*! \file i2csw.c \brief Software I2C interface using port pins. */
//*****************************************************************************
//
// File Name	: 'i2csw.c'
// Title		: Software I2C interface using port pins
// Author		: Pascal Stang
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

#include <avr/io.h>

#include "i2csw.h"


// Standard I2C bit rates are:
// 100KHz for slow speed
// 400KHz for high speed

//#define QDEL	delay(5)		// i2c quarter-bit delay
//#define HDEL	delay(10)		// i2c half-bit delay

// i2c quarter-bit delay
#define QDEL	asm volatile("nop"); asm volatile("nop"); asm volatile("nop"); asm volatile("nop"); asm volatile("nop");
// i2c half-bit delay
#define HDEL	asm volatile("nop"); asm volatile("nop"); asm volatile("nop"); asm volatile("nop"); asm volatile("nop"); asm volatile("nop"); asm volatile("nop"); asm volatile("nop"); asm volatile("nop"); asm volatile("nop");

#define I2C_SDL_A_LO      cbi( SDA_A_PORT, SDA_A)
#define I2C_SDL_A_HI      sbi( SDA_A_PORT, SDA_A)
#define I2C_SDL_B_LO      cbi( SDA_B_PORT, SDA_B)
#define I2C_SDL_B_HI      sbi( SDA_B_PORT, SDA_B)

#define I2C_SCL_LO      cbi( SCLPORT, SCL); 
#define I2C_SCL_HI      sbi( SCLPORT, SCL); 

#define I2C_SCL_TOGGLE  HDEL; I2C_SCL_HI; HDEL; I2C_SCL_LO;
#define I2C_START_A       I2C_SDL_A_LO; QDEL; I2C_SCL_LO; 
#define I2C_START_B       I2C_SDL_B_LO; QDEL; I2C_SCL_LO; 
#define I2C_STOP_A        HDEL; I2C_SCL_HI; QDEL; I2C_SDL_A_HI; HDEL;
#define I2C_STOP_B        HDEL; I2C_SCL_HI; QDEL; I2C_SDL_B_HI; HDEL;

/*
void i2ct(void)
{
	HDEL; I2C_SCL_HI; HDEL; I2C_SCL_LO;
}

void i2cstart(void)
{
	I2C_SDL_LO; QDEL; I2C_SCL_LO; 
}

void i2cstop(void)
{
	HDEL; I2C_SCL_HI; QDEL; I2C_SDL_HI; HDEL;
}


#define I2C_SCL_TOGGLE  i2ct();
#define I2C_START       i2cstart();
#define I2C_STOP        i2cstop();	
*/

UINT_AVRLIB i2cPutbyteA(u08 b)
{
	int i;
	
	for (i=7;i>=0;i--)
	{
		if ( b & (1<<i) )
			I2C_SDL_A_HI;
		else
			I2C_SDL_A_LO;			// address bit
			I2C_SCL_TOGGLE;		// clock HI, delay, then LO
	}

	I2C_SDL_A_HI;					// leave SDL HI
	// added    
	cbi(SDA_A_DDR, SDA_A);			// change direction to input on SDA line (may not be needed)
	HDEL;
	I2C_SCL_HI;					// clock back up
  	b = inb(SDA_A_PIN) & (1<<SDA_A);	// get the ACK bit

	HDEL;
	I2C_SCL_LO;					// not really ??
	sbi(SDA_A_DDR, SDA_A);			// change direction back to output
	HDEL;
	return (b == 0);			// return ACK value
}

UINT_AVRLIB i2cPutbyteB(u08 b)
{
	int i;
	
	for (i=7;i>=0;i--)
	{
		if ( b & (1<<i) )
			I2C_SDL_B_HI;
		else
			I2C_SDL_B_LO;			// address bit
			I2C_SCL_TOGGLE;		// clock HI, delay, then LO
	}

	I2C_SDL_B_HI;					// leave SDL HI
	// added    
	cbi(SDA_B_DDR, SDA_B);			// change direction to input on SDA line (may not be needed)
	HDEL;
	I2C_SCL_HI;					// clock back up
  	b = inb(SDA_B_PIN) & (1<<SDA_B);	// get the ACK bit

	HDEL;
	I2C_SCL_LO;					// not really ??
	sbi(SDA_B_DDR, SDA_B);			// change direction back to output
	HDEL;
	return (b == 0);			// return ACK value
}


u08 i2cGetbyteA(UINT_AVRLIB last)
{
	int i;
	u08 c,b = 0;
		
	I2C_SDL_A_HI;					// make sure pullups are ativated
	cbi(SDA_A_DDR, SDA_A);			// change direction to input on SDA line (may not be needed)

	for(i=7;i>=0;i--)
	{
		HDEL;
		I2C_SCL_HI;				// clock HI
	  	c = inb(SDA_A_PIN) & (1<<SDA_A);  
		b <<= 1;
		if(c) b |= 1;
		HDEL;
    	I2C_SCL_LO;				// clock LO
	}

	sbi(SDA_A_DDR, SDA_A);			// change direction to output on SDA line
  
	if (last)
		I2C_SDL_A_HI;				// set NAK
	else
		I2C_SDL_A_LO;				// set ACK

	I2C_SCL_TOGGLE;				// clock pulse
	I2C_SDL_A_HI;					// leave with SDL HI
	return b;					// return received byte
}

u08 i2cGetbyteB(UINT_AVRLIB last)
{
	int i;
	u08 c,b = 0;
		
	I2C_SDL_B_HI;					// make sure pullups are ativated
	cbi(SDA_B_DDR, SDA_B);			// change direction to input on SDA line (may not be needed)

	for(i=7;i>=0;i--)
	{
		HDEL;
		I2C_SCL_HI;				// clock HI
	  	c = inb(SDA_B_PIN) & (1<<SDA_B);  
		b <<= 1;
		if(c) b |= 1;
		HDEL;
    	I2C_SCL_LO;				// clock LO
	}

	sbi(SDA_B_DDR, SDA_B);			// change direction to output on SDA line
  
	if (last)
		I2C_SDL_B_HI;				// set NAK
	else
		I2C_SDL_B_LO;				// set ACK

	I2C_SCL_TOGGLE;				// clock pulse
	I2C_SDL_B_HI;					// leave with SDL HI
	return b;					// return received byte
}


//************************
//* I2C public functions *
//************************

//! Initialize I2C communication
void i2cInitA(void)
{
	sbi( SDA_A_DDR, SDA_A);			// set SDA as output
	sbi( SCLDDR, SCL);			// set SCL as output
	I2C_SDL_A_HI;					// set I/O state and pull-ups
	I2C_SCL_HI;					// set I/O state and pull-ups
}

void i2cInitB(void)
{
	sbi( SDA_B_DDR, SDA_B);			// set SDA as output
	sbi( SCLDDR, SCL);			// set SCL as output
	I2C_SDL_B_HI;					// set I/O state and pull-ups
	I2C_SCL_HI;					// set I/O state and pull-ups
}

//! Send a byte sequence on the I2C bus
void i2cSendA(u08 device, u08 subAddr, u08 length, u08 *data)
{
	I2C_START_A;      			// do start transition
	i2cPutbyteA(device); 		// send DEVICE address
	i2cPutbyteA(subAddr);		// and the subaddress

	// send the data
	while (length--)
		i2cPutbyteA(*data++);

	I2C_SDL_A_LO;					// clear data line and
	I2C_STOP_A;					// send STOP transition
}

//! Send a byte sequence on the I2C bus
void i2cSendB(u08 device, u08 subAddr, u08 length, u08 *data)
{
	I2C_START_B;      			// do start transition
	i2cPutbyteB(device); 		// send DEVICE address
	i2cPutbyteB(subAddr);		// and the subaddress

	// send the data
	while (length--)
		i2cPutbyteB(*data++);

	I2C_SDL_B_LO;					// clear data line and
	I2C_STOP_B;					// send STOP transition
}

//! Retrieve a byte sequence on the I2C bus
void i2cReceiveA(u08 device, u08 subAddr, u08 length, u08 *data)
{
	int j = length;
	u08 *p = data;

	I2C_START_A;					// do start transition
	i2cPutbyteA(device);			// send DEVICE address
	i2cPutbyteA(subAddr);   		// and the subaddress
	HDEL;
	I2C_SCL_HI;      			// do a repeated START
	I2C_START_A;					// transition

	i2cPutbyteA(device | READ);	// resend DEVICE, with READ bit set

	// receive data bytes
	while (j--)
		*p++ = i2cGetbyteA(j == 0);

	I2C_SDL_A_LO;					// clear data line and
	I2C_STOP_A;					// send STOP transition
}

//! Retrieve a byte sequence on the I2C bus
void i2cReceiveB(u08 device, u08 subAddr, u08 length, u08 *data)
{
	int j = length;
	u08 *p = data;

	I2C_START_B;					// do start transition
	i2cPutbyteB(device);			// send DEVICE address
	i2cPutbyteB(subAddr);   		// and the subaddress
	HDEL;
	I2C_SCL_HI;      			// do a repeated START
	I2C_START_B;					// transition

	i2cPutbyteB(device | READ);	// resend DEVICE, with READ bit set

	// receive data bytes
	while (j--)
		*p++ = i2cGetbyteB(j == 0);

	I2C_SDL_B_LO;					// clear data line and
	I2C_STOP_B;					// send STOP transition
}


