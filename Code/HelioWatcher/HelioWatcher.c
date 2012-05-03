//*****************************************************************************
// HELIOWATCHER
// 
// ECE 4760 -- Designing with Microcontrollers
// Final Project, Spring 2012
// Jason Wright (jpw97) & Jeremy Blum (jeb373)		:	
// 
//*****************************************************************************

//*****************************************************************************
// Includes
//*****************************************************************************

//--Core AVR
#include <avr/io.h>		// include I/O definitions (port names, pin names, etc)
#include <avr/interrupt.h>	// include interrupt support
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <util/delay.h>
#include <ctype.h>
#include <math.h>

//--AVRlib
#include "AVRlib\global.h"		// include our global settings
#include "AVRlib\rprintf.h"	// include printf function library
//#include "AVRlib\a2d.h"		// include A/D converter function library
//#include "AVRlib\timerx8.h"		// include timer function library (timing, PWM, etc)
#include "AVRlib\vt100.h"		// include VT100 terminal support
#include "AVRlib\i2csw.h"		// include i2c software support (stepper voltage setting)
#include "i2cmaster\i2cmaster.h"
//#include "AVRlib\i2c.h"			// include i2c hardware support (Magnetometer)
#include "AVRlib\debug.h"		// include debug functions
#include "AVRlib\uart2.h"

//--Solar Positioning Algorithm
#include "spa.h"

//--ADC Necessities
#include <avr/sleep.h>
#include <avr/power.h>

//#include "integer.h"

//--Software Delay
#include <util/delay.h>

//--Serial Debugging
#define __ASSERT_USE_STDERR
#include <assert.h>
#include "uart_mod.h"

//--GPS NMEAP Library
#include "nmeap.h"

//*****************************************************************************
// Configuration & Initialization
//*****************************************************************************

//----- UART Config ---------------------------------------------------------
// UART file descriptor
// putchar and getchar are in uart.c
FILE uart_str_0 = FDEV_SETUP_STREAM(uart_putchar_0, uart_getchar_0, _FDEV_SETUP_RW); 
FILE uart_str_1 = FDEV_SETUP_STREAM(uart_putchar_1, uart_getchar_1, _FDEV_SETUP_RW); 


//----- I2C Config ---------------------------------------------------------
#define TARGET_ADDR	0x5E

#define MAG_ADDR 0x1E
#define HMC5883L_WRITE 0x3C // write address
#define HMC5883L_READ 0x3D // read address

// local data buffer
unsigned char localBuffer[] = "testing...testing...";
unsigned char localBufferLength = 0x20;

//----- System Config ---------------------------------------------------------
int autoMode = 0; //Toggles Automatic Mode
volatile int autoFlag = 0; //Set by the ISR every minute, if in auto mode.

//data log flag
int dataLog = 0;

//----- ADC Config ---------------------------------------------------------
//General
volatile int Ain, AinLow ;                      //raw A to D number
#define Vref1 1.104
volatile float Vref = Vref1;                    //Initialize Autoscaling Vref Choice

//Panel
volatile float panelVoltage;

//Quadrature Stuff
volatile float Uvoltage;                                 //scaled input voltage
volatile float Lvoltage;                                 //scaled input voltage
volatile float Rvoltage;                                 //scaled input voltage
volatile float Dvoltage;                                 //scaled input voltage
char v_string[10];                              //scaled input voltage string to print

//Cardinal direction to ADMUX
#define ADMUXU 0b01000000
#define ADMUXL 0b01000001
#define ADMUXD 0b01000100
#define ADMUXR 0b01000101

//----- Physical Config ---------------------------------------------------------
volatile float curr_accel;

float rot;

//Use these variables to hold the currently determined optimal values for the orientation of the solar panel
double opt_rot = 0; 		//rotation from due south in degrees
int opt_screw = 0;
double opt_angle = 45; 	//angle from flat in degrees

int curr_screw = 0;

//Buffers, in case they are needed
char accel_string[10];
char rot_string[10];

#define BASE_Z -180 //z-axis measurement without being on a screw
#define SCREW_THRESHOLD 20 //threshold to detect screw

#define USE_CUSTOM_SCREW_THRESHOLDS 1

int custom_screw_thresholds[12] = {-115, -110, -100, -64, -40, -40, 100, 0, -80, -105, -120, -125};

#define SCREW_INITIAL_JUMP 75 //time for initial move, to get away from current screw
#define SCREW_BIG_JUMP 75 //not in use
#define SCREW_INCREMENT_TIME 25 //time for each move

#define MOVES_THRESHOLD 4 //Number of moves to make before considering stopping

//Raw Accelerometer values
int16_t raw_x = 0;
int16_t raw_y = 0;
int16_t raw_z = 0;

//angle vs accel calibration
//angle = m*accel + b
#define CALIB_M -0.4516
#define CALIB_B 220.56

#define MIN_ANGLE 30
#define MAX_ANGLE 52

#define OPT_ANGLE_MOVE_THRESHOLD 2 //how far away the optimum angle must be to move there

#define QUAD_ANGLE_STAY_THRESHOLD 0 //how low the quad adjust value can be before moving
#define QUAD_ANGLE_MOVE_THRESHOLD 4 //how high the quad adjust value must be to force moving

#define QUAD_ANGLE_INC_ADJUST 1000 //how much to adjust if quad forces a move

#define MIN_ROT -90
#define MAX_ROT 90

#define QUAD_ROT_STAY_THRESHOLD 0 //how low the quad adjust value can be before moving
#define QUAD_ROT_MOVE_THRESHOLD 10 //how high the quad adjust value must be to force moving

#define ANGLE_BETWEEN_SCREWS 30

#define QUAD_ADJ_SCALE 1 //how much to multiple quad adjust values by to convert to angle

//these control motor speeds
#define JACK_DELAY_US 100
#define WHEEL_DELAY_US 3000

//settings for Torch Mode!
#define TORCH_SAFETY 2
#define TORCH_THRESHOLD 5
#define TORCH_ANGLE 5
#define TORCH_ROT 0.3

int i,j; //just in case

//----- RTC Initialization ---------------------------------------------------------
volatile int UTC_hour = 12;
volatile int UTC_min = 0;
volatile int UTC_sec = 55;
volatile int UTC_msec = 0;
int UTC_offset = -4;
int UTC_month = 5;
int UTC_day = 2;
int UTC_year = 2012;

//buffers used for a few different things
char fmtTimeString[7];
char fmtDateString[7];

//flags for proper RTC counting
volatile int incSec = 0;
volatile int incMin = 0;
volatile int incHr = 0;

//These are not in use, only relevant for NREL algorithm
float sr_hour = 5;	//Sun Rise Hour
float sr_min = 0;		//Sun Rise Minute
float sr_sec = 0;		//Sun Rise Seconds
float ss_hour = 20;	//Sun Set Hour
float ss_min = 0;		//Sun Set Minute
float ss_sec = 0;		//Sun Set Second

//----- I2C Function Headers ---------------------------------------------------------

void i2cTest(void);
void i2cSlaveReceiveService(u08 receiveDataLength, u08* receiveData);
u08 i2cSlaveTransmitService(u08 transmitDataLengthMax, u08* transmitData);
void i2cMasterSendDiag(u08 deviceAddr, u08 length, u08* data);
void testI2cMemory(void);
void showByte(u08 byte);


//----- GPS ---------------------------------------------------------

nmeap_gga_t g_gga;

char gpsbuff[60];

double lati = 42.441;
double longi = -76.485;
double alti = 202.5;
uint32_t utcTime;
int numSatellites;

//From the NMEAP library
char test_vector[] = {
"$GPGGA,123519,3929.946667,N,11946.086667,E,1,08,0.9,545.4,M,46.9,M,,*4A\r\n" /* good */
"$xyz,1234,asdfadfasdfasdfljsadfkjasdfk\r\n"                                  /* junk */
"$GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68\r\n"      /* good */
"$GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*48\r\n"      /* checksum error */
};

char *pvec = test_vector;

//*****************************************************************************
// Functions -- General/Formatting
//*****************************************************************************

//string trimming
char *trim(char *s) {
    char *ptr;
    if (!s)
        return NULL;   // handle NULL string
    if (!*s)
        return s;      // handle empty string
    for (ptr = s + strlen(s) - 1; (ptr >= s) && isspace(*ptr); --ptr);
    ptr[1] = '\0';
    return s;
}

void updateTime(void)
{
	//Takes the value stored as utcTime (an int) and breaks it up into
	//component hour, minute, and second
	char timeString[11];
	sprintf(timeString, "%lu", utcTime);
	int timeStringLength = strlen(trim(timeString));

	if(dataLog==0)
	{
		fprintf(stdout, "time string: %s\r\n", timeString);
		fprintf(stdout, "time string length: %d\r\n", timeStringLength);
	}

	char minString[3];
	char secString[3];

	if(timeStringLength == 5)
	{
		char hourString[2];
		hourString[0] = timeString[0];
		UTC_hour = atoi(hourString);
		minString[0] = timeString[1];
		minString[1] = timeString[2];
		secString[0] = timeString[3];
		secString[1] = timeString[4];
	} 
	else
	{
		char hourString[3];
		hourString[0] = timeString[0];
		hourString[1] = timeString[1];
		UTC_hour = atoi(hourString);
		minString[0] = timeString[2];
		minString[1] = timeString[3];
		secString[0] = timeString[4];
		secString[1] = timeString[5];
	}

	
	UTC_min = atoi(minString);
	UTC_sec = atoi(secString);
}

void formatTime(void)
{
	//Writes UTC time components into a string.

	char hourBuff[3];
	char minBuff[3];
	char secBuff[3];

	int localHour = UTC_hour + UTC_offset;
	if (localHour < 0)
	{
		localHour += 24;
	}
	itoa(localHour, hourBuff, 10);
	itoa(UTC_min, minBuff, 10);
	itoa(UTC_sec, secBuff, 10);
	if (localHour < 10)
	{
		fmtTimeString[0] = '0';
		fmtTimeString[1] = hourBuff[0];
	}
	else
	{
		fmtTimeString[0] = hourBuff[0];
		fmtTimeString[1] = hourBuff[1];
	}
	
	if (UTC_min < 10)
	{
		fmtTimeString[2] = '0';
		fmtTimeString[3] = minBuff[0];
	}
	else
	{
		fmtTimeString[2] = minBuff[0];
		fmtTimeString[3] = minBuff[1];
	}

	if (UTC_sec < 10)
	{
		fmtTimeString[4] = '0';
		fmtTimeString[5] = secBuff[0];
	}
	else
	{
		fmtTimeString[4] = secBuff[0];
		fmtTimeString[5] = secBuff[1];
	}

}

void formatDate(void)
{
	//Writes UTC date components into a string.

	char yearBuff[4];
	char monthBuff[3];
	char dayBuff[3];

	itoa(UTC_year, yearBuff, 10);
	itoa(UTC_month, monthBuff, 10);
	itoa(UTC_day, dayBuff, 10);

	fmtDateString[4] = yearBuff[2];
	fmtDateString[5] = yearBuff[3];

	if (UTC_month < 10)
	{
		fmtDateString[0] = '0';
		fmtDateString[1] = monthBuff[0];
	}
	else
	{
		fmtDateString[0] = monthBuff[0];
		fmtDateString[1] = monthBuff[1];
	}

	if (UTC_day < 10)
	{
		fmtDateString[2] = '0';
		fmtDateString[3] = dayBuff[0];
	}
	else
	{
		fmtDateString[2] = dayBuff[0];
		fmtDateString[3] = dayBuff[1];
	}

}

//*****************************************************************************
// Functions -- GPS
//*****************************************************************************

/** simulate character by character IO */
int readchar() 
{
    int ch;
    if (*pvec == 0) {
        ch = -1;
    }
    else {
        ch = *pvec++;
    }
    return ch;
}



/** do something with the GGA data */
static void print_gga(nmeap_gga_t *gga)
{
    printf("found GPGGA message %.6f %.6f %.0f %lu %d %d %f %f\n",
            gga->latitude  ,
            gga->longitude, 
            gga->altitude , 
            gga->time     , 
            gga->satellites,
            gga->quality   ,
            gga->hdop      ,
            gga->geoid     
            );
}

//Getter functions from GGA sentence.

double gga_getLatitude(nmeap_gga_t *gga)
{
	return gga->latitude;
}

double gga_getLongitude(nmeap_gga_t *gga)
{
	return gga->longitude;
}

double gga_getAltitude(nmeap_gga_t *gga)
{
	return gga->altitude;
}

uint32_t gga_getUtcTime(nmeap_gga_t *gga)
{
	return gga->time;
}

int gga_getNumSatellites(nmeap_gga_t *gga)
{
	return gga->satellites;
}

/** called when a gpgga message is received and parsed */
static void gpgga_callout(nmeap_context_t *context,void *data,void *user_data)
{
    nmeap_gga_t *gga = (nmeap_gga_t *)data;
    
	if(dataLog == 0)
	{
	    printf("-------------callout\n");
	    print_gga(gga);
	}
}


/** do something with the RMC data */
static void print_rmc(nmeap_rmc_t *rmc)
{
    printf("found GPRMC Message %lu %c %.6f %.6f %f %f %lu %f\n",
            rmc->time,
            rmc->warn,
            rmc->latitude,
            rmc->longitude,
            rmc->speed,
            rmc->course,
            rmc->date,
            rmc->magvar
            );
}

/** called when a gprmc message is received and parsed */
static void gprmc_callout(nmeap_context_t *context,void *data,void *user_data)
{
    nmeap_rmc_t *rmc = (nmeap_rmc_t *)data;
    if(dataLog==0)
	{
    	printf("-------------callout\n");
    	print_rmc(rmc);
	}
}

static nmeap_context_t nmea;	   /* parser context */
static nmeap_gga_t     gga;		   /* this is where the data from GGA messages will show up */
static nmeap_rmc_t     rmc;		   /* this is where the data from RMC messages will show up */
static int             user_data; /* user can pass in anything. typically it will be a pointer to some user data */

void updateGPS(void)
{

	//initialize
	int             status;
	char            ch;	
	int done = 0;

	//put current GPS sentence in NMEA buffer
	pvec = trim(gpsbuff);
	strcat(pvec, "\r\n" );

	//display the sentence
	//stdout = stdin = stderr = &uart_str_0;
	if(dataLog==0)
		fprintf(stdout, "In updateGPS: %s\n---\n", pvec);

	//initialize the nmea context
	//TODO: rewrite this to return error codes instead of using uart
	status = nmeap_init(&nmea,(void *)&user_data);
	if (status != 0) {
		if(dataLog==0)
			fprintf(stdout,"nmeap_init %d\n",status);
		exit(1);
	}

	//add standard GPGGA parser
	status = nmeap_addParser(&nmea,"GPGGA",nmeap_gpgga,gpgga_callout,&gga);
	if (status != 0) {
		if(dataLog==0)
			fprintf(stdout,"nmeap_add %d\n",status);
		exit(1);
	}

	//add standard GPRMC parser
	status = nmeap_addParser(&nmea,"GPRMC",nmeap_gprmc,gprmc_callout,&rmc);
	if (status != 0) {
		if(dataLog==0)
			fprintf(stdout,"nmeap_add %d\n",status);
		exit(1);
	}

	//process input until done
	while(done == 0) {
		//get a byte at a time
		ch = readchar();
		if (ch <= 0) {
			break;
		}

		//pass it to the parser
		//status indicates whether a complete msg arrived for this byte
		// NOTE : in addition to the return status the message callout will be fired when a complete message is processed
		status = nmeap_parse(&nmea,ch);

		//process the return code
		switch(status) {
			case NMEAP_GPGGA:
				// GOT A GPGGA MESSAGE
				//fprintf(stdout,"-------------switch\n");
				//print_gga(&gga);
				//fprintf(stdout,"latitude: %f\n", gga_getLatitude(&gga));
				//fprintf(stdout,"-------------\n");
				if(gga_getLatitude(&gga) != 0.0)
					lati = gga_getLatitude(&gga);

				if(gga_getLongitude(&gga) != 0.0)
					longi = gga_getLongitude(&gga);
				
				if(gga_getAltitude(&gga) != 0.0)
					alti = gga_getAltitude(&gga);
				
				utcTime = gga_getUtcTime(&gga);
				numSatellites = gga_getNumSatellites(&gga);
				done = 1;
				break;
			case NMEAP_GPRMC:
				// GOT A GPRMC MESSAGE
				//fprintf(stdout,"-------------switch\n");
				//print_rmc(&rmc);
				//fprintf(stdout,"-------------\n");
				break;
			default:
				break;
		}
		//fprintf(stdout, "looping...");
	}

	//Only update time if we have a good GPS connection.
	//Sentences often include UTC strings that are inaccurate.
	if(numSatellites>0)
		updateTime(); 

	if(dataLog==0)
	{
		//printing values
		fprintf(stdout,"Components:\r\n");
		fprintf(stdout,"--latitude: %f\r\n",lati);
		fprintf(stdout,"--longitude: %f\r\n",longi);
		fprintf(stdout,"--altitude: %f\r\n",alti);
		fprintf(stdout,"--UTC time: %lu\r\n",utcTime);
		fprintf(stdout,"--number of satellites: %d\r\n",numSatellites); 
	}
}

//*****************************************************************************
// Functions -- Physical Math
//*****************************************************************************

//Converts acclerometer value to panel angle.
int accelToAngle(float accel)
{
	return (int)(CALIB_M*accel + CALIB_B);
}

//Converts panel angle to accelerometer value.
float angleToAccel(int angle)
{
	return (float)((angle-CALIB_B)/(CALIB_M));
}

//Converts rotation angles to screw values.
int rotToNearestScrew(float rot)
{
	int screw = 0;
	if(rot < 0)
	{
		screw = round((rot)*(-6.0/180.0))+6;
	}
	else
	{
		screw = round(rot*(6.0/180.0));
	}
	if(screw == 12)
	{
		screw = 0;
	}
	return screw;
}

//Converts screw value to rotational angle.
float screwToRot(int screw)
{
	float rot;
	if(screw <= 6)
	{
		rot = (float)(screw*(180.0/6.0));
	}
	else
	{
		rot = (float)((screw-6)*(-180.0/6.0));
	}
	return rot;
}

//Increment the screw value (i.e., one clockwise turn)
void incrementCurrScrew(void)
{
	if(curr_screw<11)
	{
		curr_screw++;
	}
	else
	{
		curr_screw = 0;
	}
}

//Decrement the screw value (i.e., one counter-clockwise turn)
void decrementCurrScrew(void)
{
	if(curr_screw == 0)
	{
		curr_screw = 11;
	}
	else
	{
		curr_screw--;
	}
}

//Calculates screw distance between two screws via a CW path
int distanceBetweenScrewsCW(int screw1, int screw2)
{
	if(screw2 >= screw1)
	{
		return (screw2 - screw1);
	}
	else
	{
		return ((screw2+12)-screw1);
	}
}

//Calculates screw distance between two screws via a CCW path
int distanceBetweenScrewsCCW(int screw1, int screw2)
{
	if(screw2 <= screw1)
	{
		return (screw1 - screw2);
	}
	else
	{
		return ((screw1+12)-screw2);
	}
}

//Gets adjustment factor given two quadrature voltages
float getAdjDeg(float voltage1, float voltage2)
{
	return (float)QUAD_ADJ_SCALE*((180.0*voltage1/(voltage1+voltage2))-90.0);
}

//*****************************************************************************
// Functions -- Sensors
//*****************************************************************************

//Reads data from the magnetometer.
float getRotation()
{
	
	i2c_start_wait(HMC5883L_WRITE);
	i2c_write(0x03); //set pointer to X-axis MSB
	i2c_stop();

	i2c_rep_start(HMC5883L_READ); 

	raw_x = ((uint8_t)i2c_readAck())<<8;
	raw_x |= i2c_readAck();

	raw_z = ((uint8_t)i2c_readAck())<<8;
	raw_z |= i2c_readAck();

	raw_y = ((uint8_t)i2c_readAck())<<8;
	raw_y |= i2c_readNak();

	i2c_stop();

	//Calculate deviation from South in Degrees
	//Negative is CCW, Positive is CW
	//A Heading of 0 or 360 degrees  means the panel is pointing North
	//A heading of 180 degrees means the panel is pointing South.
	
	if(dataLog==0)
		fprintf(stdout, "x: %d\r\ny: %d\r\nz: %d\r\n", raw_x,raw_y,raw_z);
	_delay_ms(500);

	//raw_x = (raw_x-8.5)*(255/71.5);
	//raw_y = (raw_y+102)*(255/102);

	//float heading = atan2((double)raw_x,(double)raw_y)* 180 / 3.14159265 + 180;
	float heading = atan2((double)raw_x,(double)raw_y);
	if (heading<0) heading += 2*M_PI;
	heading = heading *180/M_PI;

	//0 is heading south
	float rotation = 180.0 - heading;

	return rotation;
}

//Reads data from the accelerometer.
void updateAccel(void)
{
	sleep_cpu();
    curr_accel = (float)Ain ;
	_delay_ms(100);
	
}

void updateRot(void)
{

	rot = getRotation();
	if(dataLog==0)
	{
		dtostrf(rot, 6, 3, rot_string);    
	    fprintf(stdout, "Current Orientation %s\r\n", rot_string);
	}
	
}

//Set up the magnetometer.
void setupMag()
{

	i2c_start(HMC5883L_WRITE);
	i2c_write(0x00); // set pointer to CRA
	i2c_write(0x70); // write 0x70 to CRA
	i2c_stop();

	i2c_start(HMC5883L_WRITE);
	i2c_write(0x01); // set pointer to CRB
	i2c_write(0xA0);
	i2c_stop();

	i2c_start(HMC5883L_WRITE);
	i2c_write(0x02); // set pointer to measurement mode
	i2c_write(0x00); // continous measurement
	i2c_stop();

}

//Update values from the quadrature
void updateQuad(void)
{

	ADCSRB &= ~0b00001000; //disable highest mux bit

	ADMUX = ADMUXU;

    sleep_cpu();
    Uvoltage = (float)Ain ;
	_delay_ms(100);

    Uvoltage = 1000*(Uvoltage/1024.0)*Vref ;   //(fraction of full scale)*Vref

	ADMUX = ADMUXL;

    sleep_cpu();
    Lvoltage = (float)Ain ;
	_delay_ms(100);

    Lvoltage = 1000*(Lvoltage/1024.0)*Vref ;   //(fraction of full scale)

	ADMUX = ADMUXR;

    sleep_cpu();
    Rvoltage = (float)Ain ;

	_delay_ms(100);

    Rvoltage = 1000*(Rvoltage/1024.0)*Vref ;   //(fraction of full scale)  

	ADMUX = ADMUXD;

    sleep_cpu();
    Dvoltage = (float)Ain ;

	_delay_ms(100);

    Dvoltage = 1000*(Dvoltage/1024.0)*Vref ;   //(fraction of full scale)

	ADCSRB |= 0b00001000; //enable highest mux bit
	ADMUX = 0b01000110; //go back to accelerometer config

}

//Updates panel voltage.
void updatePanel(void)
{
	ADMUX = 0b01000111; //pick panel
	sleep_cpu();
    panelVoltage = (float)(6.6*Ain*5/1024) ; //It's being passed through a voltage divider, see documentation.
	_delay_ms(100);
	ADMUX = 0b01000110; //go back to accelerometer config
}

//*****************************************************************************
// Functions -- Movement
//*****************************************************************************

//These do what they say: move the stepper motors.
//How these operate will depend on JACK_DELAY_US and WHEEL_DELAY_US, so be careful.

void goDown(int duration)
{
	u08 buf[1];
	buf[0] = 0x7F;   
	i2cSendB(TARGET_ADDR, 0, 1, buf);
	PORTG &= ~0b00000100; //enable stepper
	PORTA |= 0b01000000; //set direction down
	for(i=0; i<duration; i++) {
		for (j=0; j<5; j++) {
			PORTA |= 0b10000000;

			_delay_us(JACK_DELAY_US);

			PORTA &= ~0b10000000;

			_delay_us(JACK_DELAY_US);
		}
	}
	PORTG |= 0b00000100; //disable stepper
	i2cInitB();
	buf[0] = 0x00;   //zero vref
	i2cSendB(TARGET_ADDR, 0, 1, buf);
}

void goUp(int duration)
{
	u08 buf[1];
	buf[0] = 0x7F;   
	i2cSendB(TARGET_ADDR, 0, 1, buf);
	PORTG &= ~0b00000100; //enable stepper
	PORTA &= ~0b01000000; //set direction up
	for(i=0; i<duration; i++) {
		for (j=0; j<5; j++) {
			PORTA |= 0b10000000;

			_delay_us(JACK_DELAY_US);

			PORTA &= ~0b10000000;

			_delay_us(JACK_DELAY_US);
		}
	}
	PORTG |= 0b00000100; //disable stepper
	i2cInitB();
	buf[0] = 0x00;   //zero vref
	i2cSendB(TARGET_ADDR, 0, 1, buf);
}

void goToAngle(int valNum)
{
	float accel_wanted = angleToAccel(valNum);

	if(dataLog==0)
	{
		dtostrf(accel_wanted, 6, 3, accel_string);    
	    fprintf(stdout, "Looking for accel value %s\r\n", accel_string);
	}
	
	
	if ((valNum<=MAX_ANGLE)&&(valNum>=MIN_ANGLE))
	{
		if(curr_accel < accel_wanted)
		{
			while(curr_accel < accel_wanted)
			{
				goDown(1000);
				updateAccel();
			}	
		}
		else if(curr_accel >= accel_wanted)
		{
			while(curr_accel > accel_wanted)
			{
				goUp(1000);
				updateAccel();
			}
		}
	}
	else
	{
		fprintf(stdout, "I'm sorry Dave, I'm afraid I can't do that...\r\n");
	}
}

void goLeft(int duration)
{
	u08 buf[1];
	buf[0] = 0x7F;   
	i2cSendA(TARGET_ADDR, 0, 1, buf);
	PORTA &= ~0b00010000; //enable stepper
	PORTA &= ~0b00000100;
	for(i=0; i<duration; i++) {
		for (j=0; j<5; j++) {
			PORTA |= 0b00001000;

			_delay_us(WHEEL_DELAY_US);

			PORTA &= ~0b00001000;

			_delay_us(WHEEL_DELAY_US);
		}
	}
	PORTA |= 0b00010000; //disable stepper
	i2cInitA();
	i2cSendA(TARGET_ADDR, 0, 1, buf);
	buf[0] = 0x00;   //zero vref
}

void goRight(int duration)
{
	u08 buf[1];
	buf[0] = 0x7F;   
	i2cSendA(TARGET_ADDR, 0, 1, buf);
	PORTA &= ~0b00010000; //enable stepper
	PORTA |= 0b00000100;
	for(i=0; i<duration; i++) {
		for (j=0; j<5; j++) {
			PORTA |= 0b00001000;

			_delay_us(WHEEL_DELAY_US);

			PORTA &= ~0b00001000;

			_delay_us(WHEEL_DELAY_US);
		}
	}
	PORTA |= 0b00010000; //disable stepper
	i2cInitA();
	buf[0] = 0x00;   //zero vref
	i2cSendA(TARGET_ADDR, 0, 1, buf);
}

void goToScrewCW(void)
{
	if(dataLog==0)
		fprintf(stdout, "starting screw: %u\r\n", curr_screw);
	
	int numMoves = 0;
	if(curr_screw == 0 || curr_screw == 3 || curr_screw == 6 || curr_screw == 9)
	{
		goLeft(SCREW_BIG_JUMP);
	}
	else
	{
		goLeft(SCREW_INITIAL_JUMP);
	}
	updateRot();
	int activeScrewThresh;
	int nextScrew;
	if(curr_screw == 11)
		nextScrew = 0;
	else
		nextScrew = curr_screw+1;
	if(USE_CUSTOM_SCREW_THRESHOLDS)
		activeScrewThresh = custom_screw_thresholds[nextScrew];
	else
		activeScrewThresh = (BASE_Z + SCREW_THRESHOLD);
	while (raw_z < activeScrewThresh || numMoves < MOVES_THRESHOLD)
	{
		goLeft(SCREW_INCREMENT_TIME);
		updateRot();
		numMoves++;
	}
	incrementCurrScrew();
	
	if(dataLog==0)
		fprintf(stdout, "final screw: %u\r\n", curr_screw);
}

void goToScrewCCW(void)
{
	if(dataLog==0)
		fprintf(stdout, "starting screw: %u\r\n", curr_screw);
	
	int numMoves = 0;
	if(curr_screw == 0 || curr_screw == 3 || curr_screw == 6 || curr_screw == 9)
	{
		goRight(SCREW_BIG_JUMP);
	}
	else
	{
		goRight(SCREW_INITIAL_JUMP);
	}
	updateRot();
	int activeScrewThresh;
	int nextScrew;
	if(curr_screw == 0)
		nextScrew = 11;
	else
		nextScrew = curr_screw-1;
	if(USE_CUSTOM_SCREW_THRESHOLDS)
		activeScrewThresh = custom_screw_thresholds[nextScrew];
	else
		activeScrewThresh = (BASE_Z + SCREW_THRESHOLD);
	while (raw_z < activeScrewThresh || numMoves < MOVES_THRESHOLD)
	{
		goRight(SCREW_INCREMENT_TIME);
		updateRot();
		numMoves++;
	}
	decrementCurrScrew();
	
	if(dataLog==0)
		fprintf(stdout, "final screw: %u\r\n", curr_screw);
}

void goToScrew(int screw)
{
	if(dataLog==0)
		fprintf(stdout, "on a journey from screw %u to screw %u\r\n", curr_screw, screw);
	
	if (screw >=0 && screw < 12)
	{
		if (distanceBetweenScrewsCW(curr_screw, screw) < distanceBetweenScrewsCCW(curr_screw, screw))
		{
			while (curr_screw != screw)
			{
				goToScrewCW();
			}
		}
		else
		{
			while(curr_screw != screw)
			{
				goToScrewCCW();
			}
		}
	}
	else
	{
		fprintf(stdout, "I'm sorry Dave, I'm afraid I can't do that...\r\n");
	}
}

void goToRot(float rot_wanted)
{
	
	if ((rot_wanted<=MAX_ROT)&&(rot_wanted>=MIN_ROT))
	{
		goToScrew(rotToNearestScrew(rot_wanted));

		//Old version using the accelerometer
		/*if(rot < rot_wanted)
		{
			while(rot < rot_wanted)
			{
				goLeft(50);
				updateRot();
			}	
		}
		else if(rot >= rot_wanted)
		{
			while(rot > rot_wanted)
			{
				goRight(50);
				updateRot();
			}
		}*/
	}
	else
	{
		fprintf(stdout, "I'm sorry Dave, I'm afraid I can't do that...\r\n");
	}
}

//*****************************************************************************
// Functions -- Solar Position Algorithms
//*****************************************************************************

// Function 1 -- This is the NREL algorithm (see documentation)
// We had trouble getting this to work on AVR, so it is not in use.
 
//Determines the optimal angle and rotation for the solar panel, given GPS information!
//Outputs are saved into global variables, for ease of use.
void optimize(void)
{

	spa_data spa;  //declare the SPA structure
    int result;
    float sr_min, ss_min, sr_sec, ss_sec;

	//Daylight Savings Time is Observed in Most of Europe and the United States/Canada, but not the Rest of the World.
	//To do a rough compensation for this, draw a rectangle around the US/Canada and Europe and Adjust for DST in those places.
	//I'm going to approximate the DST Dates:
	//DST Starts in March.  DST Ends in November. (US/Canada)
	//DST Starts in April.  DST Ends in November. (EU)
	int DST = 0;
	
	//US/Canada Rectangle (-20 to -180 Longitude, 20 to 100 Latitude)
	if (-180 < longi && longi < -20 && 20 < lati && lati < 100 && 03 < UTC_month && UTC_month < 11)
		DST = 1;
	
	//Europe Rectangle (20 to 40 Longitude, 35 to 80 Latitude)
	if (20 < longi && longi < 40 && 35 < lati && lati < 80 && 04 < UTC_month && UTC_month < 11 )
		DST = 1;
	
	
    //Now, we approximate the timezone
    int time_offset = ((int) round(longi/15.0)) + DST;

    spa.year          = UTC_year;
    spa.month         = UTC_month;
    spa.day           = UTC_day;
    spa.hour          = UTC_hour + time_offset;
    spa.minute        = UTC_min;
    spa.second        = UTC_sec;
    spa.timezone      = time_offset;
    spa.delta_t       = 67;		//Default
    spa.longitude     = longi;
    spa.latitude      = lati;
    spa.elevation     = alti;
    spa.pressure      = 820; 	//Default
    spa.temperature   = 30; 	//Default (for now)
    spa.slope         = 0; 		//default
    spa.azm_rotation  = 0; 		//facing south
    spa.atmos_refract = 0.5667; //default
    spa.function      = SPA_ALL;
    

    //call the SPA calculate function and pass the SPA structure

    result = spa_calculate(&spa);

    if (result == 0)  //check for SPA errors
    {

		//Save key info into optimal global variables.
		opt_rot = spa.azimuth-180; 		//rotation from due south in degrees
		opt_screw = rotToNearestScrew(opt_rot);
		opt_angle = 90 - (90 - spa.zenith); 	//angle from flat in degrees

		sr_min = 60.0*(spa.sunrise - (int)(spa.sunrise));
        sr_sec = 60.0*(sr_min - (int)sr_min);

		ss_min = 60.0*(spa.sunset - (int)(spa.sunset));
        ss_sec = 60.0*(ss_min - (int)ss_min);

		
		//Save some more info to global variables (sunrise and set)
		sr_hour = (int)(spa.sunrise);	//Sun Rise Hour
		ss_hour = (int)(spa.sunset);	//Sun Set Hour

		UTC_offset = (int)spa.timezone;

		if(dataLog==0)
		{
	        //display the results inside the SPA structure
			fprintf(stdout,"USEFUL INFO\n");
			fprintf(stdout,"Time:                              %02d:%02d:%02d Local Time\n", (int)(spa.hour), (int)(spa.minute), (int)(spa.second));
			fprintf(stdout,"Rot from S. (Neg = CCW, Pos = CW): %.6f degrees\n",opt_rot);
			fprintf(stdout,"Panel Angle from Flat:             %.6f degrees\n",opt_angle);
		
	        fprintf(stdout,"Sunrise:                           %02d:%02d:%02d Local Time\n", (int)(spa.sunrise), (int)sr_min, (int)sr_sec);

        
	        fprintf(stdout,"Sunset:                            %02d:%02d:%02d Local Time\n", (int)(spa.sunset), (int)ss_min, (int)ss_sec);

			fprintf(stdout,"\nOTHER RANDOM INFO\n");
			fprintf(stdout,"Julian Day:                %.6f\n",spa.jd);
			fprintf(stdout,"Time Offset:               %d\n", (int)spa.timezone);
		
			if (DST == 1)
			fprintf(stdout,"DST:                       Daylight Savings is Active\n");
			else
			fprintf(stdout,"DST:                       Daylight Savings is Inactive\n\n");
	        fprintf(stdout,"L:                         %.6e degrees\n",spa.l);
	        fprintf(stdout,"B:                         %.6e degrees\n",spa.b);
	        fprintf(stdout,"R:                         %.6f AU\n",spa.r);
	        fprintf(stdout,"H:                         %.6f degrees\n",spa.h);
	        fprintf(stdout,"Delta Psi:                 %.6e degrees\n",spa.del_psi);
	        fprintf(stdout,"Delta Epsilon:             %.6e degrees\n",spa.del_epsilon);
	        fprintf(stdout,"Epsilon:                   %.6f degrees\n",spa.epsilon);
	        fprintf(stdout,"Incidence:                 %.6f degrees\n",spa.incidence);
			fprintf(stdout,"Zenith:                    %.6f degrees\n",spa.zenith);
			fprintf(stdout,"Degrees Above Horizon:     %.6f degrees\n",90 - spa.zenith);
		}

    } else fprintf(stdout,"SPA Error Code: %d\n", result);

}

// Function 2 -- Goldstein's algorithm.
// This is the one that is actually used.

void optimize2()
{

/*
	the following function approximates the sun's altitude and azimuth within about 1 degree of accuracy
*/


   /*
      calculate the Sun's altitude and azimuth	
		inputs: 'lat'itude, 'lon'gitude, dt_type('u'|'l'), 'mo'nth, 'da'y, hour, minute
		dt_type contains 'u'tc date/time, or 'l'ocal date/time flag indicating whether mo, da, hour and minute are utc or local
	   outputs: 'alt'itude, 'az'imuth 
		code based upon Prof. Richard B. Goldstein's sun position calculator at http://www.providence.edu/mcs/rbg/java/sungraph.htm
		Appears to give fairly accurate results for the next 10 years or so
	*/

	float ti;
	int local_hour;
	int local_day = UTC_day;
	int local_month = UTC_month;
	
	/* calculate local date/time from utc date/time */
	float time_offset = round(longi / 15); //longitude -> hours
	local_hour = UTC_hour + time_offset;

	//adjust hour, day, month as needed (ignore years and seconds)...  
	if (local_hour < 0){
		local_hour += 24;
		local_day--;
		if (local_day < 1){
			local_month--;
			if (local_month < 1) local_month = 12;
			local_day = (local_month==4||local_month==6||local_month==9||local_month==11)?30:(local_month==1||local_month==3||local_month==5||local_month==7||local_month==10)?31:28;
		}
	}
	else if (local_hour > 23){
		local_hour -= 24;
		local_day++;
		if (local_day > (local_month==4||local_month==6||local_month==9||local_month==11)?30:(local_month==1||local_month==3||local_month==5||local_month==7||local_month==10)?31:28){
			local_month++;
			if (local_month > 12) local_month = 1;
			local_day = 1;
		}
	}
	
	//total hours and minutes and adjust for +/- offset from noon...
   ti = (local_hour + UTC_min / 60) - 12;
	  
    float pi180=M_PI/180;
    float adjtime;
    float za[] = {-0.5,30.5,58.5,89.5,119.5,150.5,180.5,211.5,242.5,272.5,303.5,333.5}; // days from jan at noon
    float zi;
    float zzi;
    float cth; // cosine of latitude
    float sth; // sine of latitude
    float cph; 
    float sph;
    float cti;
    float sti;
    float x;
    float y;
    float loc;
    float phi;
    float sin_tau, cos_tau;

    loc = round(longi / 15) * 15; // 
    adjtime = (longi - loc) / 40; // offset 
    zi = za[(int)local_month - 1];       // 
    zzi = 360 * (zi + 0.5 + local_day - 82) / 365; // 
    cos_tau = cos(zzi * pi180);
    sin_tau = sin(zzi * pi180);
    phi = acos(cos_tau * cos_tau + sin_tau * sin_tau * cos(23.45 * pi180)); // formula for sun declination (varies +/- 23.45 deg. over a year)
    phi = round(1000 * phi / pi180) / 1000; //
    if (sin_tau < 0){phi = -phi;}// sign +/- depends on the time of year   
    ti = ti * 15; // hours +/- offset from noon to degrees from Prime Meridian
    cth = cos(lati * pi180);
    sth = sin(lati * pi180);
    cph = cos(phi * pi180);
    sph = sin(phi * pi180);
    cti = cos(ti * pi180);

    //altitude = sin-1(sin theta * sin phi + cos theta * cos phi * cos tau)

    opt_angle = sth * sph + cth * cph * cti;
    opt_angle = asin(opt_angle) / pi180;
    opt_angle = round(1000 * opt_angle) / 1000;
	opt_angle = 90-opt_angle;

    //azimuth = tan-1(-x'/y')=tan-1(cos phi sin tau/(cos theta sin phi - sin theta cos phi cos tau))

    sti = sin(ti * pi180);
    x = -cph * sti;
    y = cth * sph - sth * cph * cti;
    opt_rot = 90 - atan2(y, x) / pi180;
    if(opt_rot < 0) opt_rot = opt_rot + 360;
    opt_rot = round(1000 * opt_rot) / 1000;
	opt_rot = opt_rot-180;

	opt_screw = rotToNearestScrew(opt_rot);
	
	if(dataLog==0)
		fprintf(stdout, "results: opt_rot %f, opt_angle %f\r\n", opt_rot, opt_angle);

}

//*****************************************************************************
// Functions -- Automatic Mode/Data Logger
//*****************************************************************************

// This function is called during automatic mode, and uses acquired data to make
// a decision about whether to move and how

void considerMoving(void)
{
	//fprintf(stdout, "considering moving...\r\n");
	if(dataLog==0)
		fprintf(stdout, "ROTATION -- opt screw %i, curr screw %i, quad adj %f\r\n",opt_screw,curr_screw,(double)getAdjDeg(Lvoltage,Rvoltage));
	
	if((opt_screw != curr_screw) && (abs(getAdjDeg(Lvoltage,Rvoltage))>=QUAD_ROT_STAY_THRESHOLD))
	{
		if(screwToRot(opt_screw) < MIN_ROT)
			goToScrew(rotToNearestScrew(MIN_ROT));
		else if(screwToRot(opt_screw) > MAX_ROT)
			goToScrew(rotToNearestScrew(MAX_ROT));
		else
			goToScrew(opt_screw);
	}
	else if(abs(getAdjDeg(Lvoltage,Rvoltage))>QUAD_ROT_MOVE_THRESHOLD)
	{
		if(getAdjDeg(Lvoltage,Rvoltage)>0)
		{
			if(screwToRot(curr_screw)<=(MAX_ROT-ANGLE_BETWEEN_SCREWS))
				goToScrewCW();
		}
		else
		{
			if(screwToRot(curr_screw)>=(MIN_ROT+ANGLE_BETWEEN_SCREWS))
				goToScrewCCW();
		}
	}

	if(dataLog==0)
		fprintf(stdout, "ANGLE -- opt angle %f, curr angle %i, quad adj %f\r\n",opt_angle,accelToAngle(curr_accel),(double)getAdjDeg(Dvoltage,Uvoltage));
	
	if((abs(opt_angle-accelToAngle(curr_accel))>OPT_ANGLE_MOVE_THRESHOLD) && (abs(getAdjDeg(Dvoltage,Uvoltage))>QUAD_ANGLE_STAY_THRESHOLD))
	{
		if(opt_angle < MIN_ANGLE)
			goToAngle(MIN_ANGLE);
		else if(opt_angle > MAX_ANGLE)
			goToAngle(MAX_ANGLE);
		else
			goToAngle(opt_angle);
	}
	else if(abs(getAdjDeg(Dvoltage,Uvoltage))>QUAD_ANGLE_MOVE_THRESHOLD)
	{
		if(getAdjDeg(Dvoltage,Uvoltage)>0)
			goUp(QUAD_ANGLE_INC_ADJUST);
		else
			goDown(QUAD_ANGLE_INC_ADJUST);
	}
}

// Sends a properly formatted string to the data logger, see documentation.

void printRecord(void)
{
	formatTime();
	formatDate();
	//updateAccel();
	//updatePanel();
	//updateQuad();

	double quad_rot_adj = (double)getAdjDeg(Lvoltage,Rvoltage);
	double quad_angle_adj = (double)getAdjDeg(Dvoltage,Uvoltage);

	fprintf(stdout, "%s,%s,%f,%f,", fmtDateString,fmtTimeString,opt_rot,opt_angle);
	fprintf(stdout, "%f,%i,%f,%f,%f;\r\n", (double)screwToRot(curr_screw),accelToAngle(curr_accel),(double)panelVoltage,quad_rot_adj,quad_angle_adj);
}

//*****************************************************************************
// Interrupt Service Routines
//*****************************************************************************

ISR (TIMER1_OVF_vect)
{
	fprintf(stdout, "Overflow!!!\n");
	
}

// Performs all RTC operations

ISR (TIMER1_COMPA_vect)
{

	//fprintf(stdout,"Tick %u\n\r", TCNT1);

	
	if(UTC_msec < 999)
	{
		UTC_msec++;
	} 
	else
	{
		UTC_msec = 0;
		incSec = 1;
	}
	if(incSec == 1)
	{
		incSec = 0;
		if(UTC_sec < 59)
		{
			UTC_sec++;
		}
		else
		{
			UTC_sec = 0;
			incMin = 1;
		}
		//fprintf(stdout, "second: %d\r\n", UTC_sec);
	}
	if(incMin == 1)
	{
		

		//movement
		if(autoMode == 1)
			autoFlag = 1;

		incMin = 0;
		if(UTC_min < 59)
		{
			UTC_min++;
		}
		else
		{
			UTC_min = 0;
			incHr = 1;
		}
	}
	if(incHr == 1)
	{
		incHr = 0;
		if(UTC_hour < 23)
		{
			UTC_hour++;
		}
		else
		{
			UTC_hour = 0;
		}
	}
}  

ISR (ADC_vect)
{
    //program ONLY gets here when ADC done flag is set
    //when reading 10-bit values
    //you MUST read the low byte first
    AinLow = (int)ADCL;
    Ain = (int)ADCH*256;
    Ain = Ain + AinLow;
}

//*****************************************************************************
// Main
//*****************************************************************************

int main(void)
{
	//init UARTs
	uart_init_0();
	uart_init_1();
	stdout = stdin = stderr = &uart_str_0;
	
	if(dataLog==0)
		fprintf(stdout,"Starting...\n\r");
	
	//set DDRs

	DDRG=0xff;
	DDRA=0xff;

	//Setting Stepper A Current to zero
	i2cInitA();
	u08 buf[1];      // a buffer can store 8 bytes 
	buf[0] = 0x00;   // hours = 0x19 in the first byte 
	i2cSendA(TARGET_ADDR, 0, 1, buf);

	//Setting Stepper B Current to zero
	i2cInitB();
	i2cSendB(TARGET_ADDR, 0, 1, buf);

	//Setup the Magnetometer
	i2c_init_HW();
	setupMag();

	PORTG |= 0b00000100; //disable jack stepper
	PORTA |= 0b00010000; //disable wheel stepper

	//init the A to D converter 

    ADMUX = 0b01000110; //sets voltage ref, right adjust result, sets most of MUX
    ADCSRA = ((1<<ADEN) | (1<<ADIE)) + 7 ; //enables the ADC, sets prescaler to 127, sets int enable
	ADCSRB |= 0b00001000; //one of the MUX bits is in here
	DIDR0 = ((1<<ADC5D) | (1<<ADC4D) | (1<<ADC1D) | (1<<ADC0D)); //disabling digital inputs
	DIDR2 = (1<<ADC15D);

    SMCR = (1<<SM0) ; // sleep -- choose ADC mode

	sleep_enable();

	//Set up 16-bit TIMER1 for 1mSec ticks for RTC
	TIMSK1 = 3;						//turn on timer 1 cmp match ISR and overflow ISR
	OCR1AH = 1999U >> 8; 			//sets OCR1A to 2000 ticks
	OCR1AL = 1999U & 0xff;
	TCCR1B = ((1<<CS11) | (1<<WGM12));	//clock prescalar to 8 and set to CTC mode
	
	sei(); //crank up them interrupts
	
	//Get Initial GPS data
	fscanf(&uart_str_1, "%s", gpsbuff);

	if(dataLog==0)
	{
		//Welcome
		fprintf(stdout, "Received GPS sentence: %s\n---\n", gpsbuff);

		fprintf(stdout, "*---Manual Operation---*\r\n");
		fprintf(stdout, "Welcome to HelioWatcher -- type help for a list of commands.\r\n");
	}
	else
	{
		//do some initialization for the data logger
		updateAccel();
		updateGPS();
		optimize2();
		autoMode = 1;

	}

	//Initialize variables used for user input
	char *spaceNum;
	int valNum ;
	float valFloat;
	char valStr[18];
	char input[20] ;
	char cmd_raw[18];
	char *cmd;
	
	while(1) {
		if(dataLog==0)
		{
			 //Print a prompt, and wait for input from the User.
			fprintf(stdout, ">") ;
	
			
			memset(input, 0, sizeof(input)); //Clear previous input
			fgets(input, 20, stdin); //Get the actual input
			spaceNum = strchr(input,' '); //Find space delimiter
			memset(valStr, 0, sizeof(valStr)); //Clear previous argument
	  		strncpy(valStr, spaceNum, strlen(spaceNum)); //Copy argument to valStr

			memset(cmd_raw, 0, sizeof(cmd_raw)); //Clear previous command
			strncpy(cmd_raw, input, strlen(input)-strlen(spaceNum)); //Copy command to cmd_raw
			cmd = trim(cmd_raw); //clean it

			//Convert to various formats
			valNum = atoi(trim(valStr));
			valFloat = atof(trim(valStr));

			//Big long pseudo-switch
			if (strcmp(cmd,"help") == 0)
			{
				fprintf(stdout, "Type a command (followed by an argument, if necessary) using the following syntax:\r\n");
				fprintf(stdout, "accel -- read accelerometer value (void)\r\n");
				fprintf(stdout, "angle -- go to angle (degrees)\r\n");
				fprintf(stdout, "angletoscrew -- convert angle to screw (degrees)\r\n");
				fprintf(stdout, "auto -- go into auto mode (void) **can't leave this\r\n");
				fprintf(stdout, "ccw -- go to nearest CCW screw (void)\r\n");
				fprintf(stdout, "compass -- get compass rotation (void) **DEPRECATED\r\n");
				fprintf(stdout, "cw -- go to nearest CW screw (void)\r\n");  
				fprintf(stdout, "data -- go into data logging mode (void) **can't leave this\r\n");
				fprintf(stdout, "date -- display the current date (void)\r\n");
				fprintf(stdout, "down -- move down (ms)\r\n");
				fprintf(stdout, "gotoopt -- move to optimal position (void) **must run opt first\r\n");
				fprintf(stdout, "gps -- read GPS values (void)\r\n");
				fprintf(stdout, "help -- list HelioWatcher shell functions\r\n");
				fprintf(stdout, "left -- move left (ms)\r\n");
				fprintf(stdout, "opt -- get optimal positioning info based on current GPS data (void) **must run gps first\r\n");
				fprintf(stdout, "panel -- display the panel voltage (void)\r\n");
				fprintf(stdout, "quad -- view quadrature values (void)\r\n");
				fprintf(stdout, "record -- print string that would be sent to data logger (void)\r\n");
				fprintf(stdout, "right -- move right (ms)\r\n");
				fprintf(stdout, "rot -- go to rot (degrees) **DEPRECATED\r\n");
				fprintf(stdout, "screw -- go to arbitrary screw (screw num)\r\n");
				fprintf(stdout, "setday -- set UTC day (int)\r\n");
				fprintf(stdout, "sethour -- set UTC hour (int)\r\n");
				fprintf(stdout, "setmin -- set UTC min (int)\r\n");
				fprintf(stdout, "setmonth -- set UTC month (int)\r\n");
				fprintf(stdout, "setsec -- set UTC sec (int)\r\n");
				fprintf(stdout, "setyear -- set UTC year (int)\r\n");
				fprintf(stdout, "time -- display the current time (void)\r\n");
				fprintf(stdout, "torch -- orient based on quadrature demo (sensitivity 0-100) **can't leave this\r\n");
				fprintf(stdout, "up -- move up (ms)\r\n");					
			}
			else if (strcmp(cmd,"opt") == 0)
			{
				optimize2();
			}
			else if (strcmp(cmd,"quad") == 0)
			{
				updateQuad();

				//print values
				dtostrf(Uvoltage, 6, 3, v_string);    
				fprintf(stdout, "U voltage: %s\r\n", v_string); 
				dtostrf(Lvoltage, 6, 3, v_string);    
				fprintf(stdout, "L voltage: %s\r\n", v_string); 
				dtostrf(Rvoltage, 6, 3, v_string);    
				fprintf(stdout, "R voltage: %s\r\n", v_string);
				dtostrf(Dvoltage, 6, 3, v_string);    
				fprintf(stdout, "D voltage: %s\r\n", v_string);

			}
			else if (strcmp(cmd,"torch") == 0)
			{
				//Torch mode -- Hold up a "torch" to the panel to move it in real time.
				int UDaxis;
				int UDchange;
				int LRaxis;
				int LRchange;
				int currAngle;

				while(1)
				{

					//Update all the values, and print them
					updateAccel();
					currAngle = accelToAngle(curr_accel);
					fprintf(stdout, "currAngle %u\r\n", currAngle);

					updateQuad();
					UDaxis = Uvoltage-Dvoltage;
					UDchange = abs(UDaxis)*valNum;

					dtostrf(UDaxis, 6, 3, v_string);    
					fprintf(stdout, "UD axis: %s\r\n", v_string);

					LRaxis = Lvoltage-Rvoltage;
					LRchange = abs(LRaxis)*valNum;

					dtostrf(LRaxis, 6, 3, v_string);    
					fprintf(stdout, "LR axis: %s\r\n", v_string);

					//Change angle
					if(abs(UDaxis)>TORCH_THRESHOLD)
					{
						if (UDaxis>0)
						{
							if(currAngle>MIN_ANGLE+TORCH_SAFETY)
								goDown(UDchange*TORCH_ANGLE);
						}
						else
						{
							if(currAngle<MAX_ANGLE-TORCH_SAFETY)
								goUp(UDchange*TORCH_ANGLE);
						}
					}

					//Change rotation
					if(abs(LRaxis)>TORCH_THRESHOLD)
					{
						if(LRaxis>0)
						{
							goLeft(LRchange*TORCH_ROT);
						}
						else
						{
							goRight(LRchange*TORCH_ROT);
						}
					}
				}
			}
			else if (strcmp(cmd,"angletoscrew") == 0)
			{
				int convertedScrew = rotToNearestScrew(valFloat);
				fprintf(stdout, "screw is %u\r\n", convertedScrew);
			}
			else if (strcmp(cmd,"screw") == 0)
			{
				goToScrew(valNum);
			}
			else if (strcmp(cmd,"cw") == 0)
			{
				goToScrewCW();
			}
			else if (strcmp(cmd,"ccw") == 0)
			{
				goToScrewCCW();
			}
			else if (strcmp(cmd,"compass") == 0)
			{
				updateRot(); //updates rot
				curr_screw = rotToNearestScrew(rot);
				fprintf(stdout, "Nearest screw: %d\n\r", curr_screw);
			}
			else if (strcmp(cmd,"gotoopt") == 0)
			{
				//Moves to optimum values of angle and rot, if possible.

				fprintf(stdout, "going to angle ");
				if(opt_angle < MIN_ANGLE) 
				{
					goToAngle(MIN_ANGLE);
					fprintf(stdout, "%u", MIN_ANGLE);
				}
				else if(opt_angle > MAX_ANGLE) 
				{
					goToAngle(MAX_ANGLE);
					fprintf(stdout, "%u", MAX_ANGLE);
				}
				else
				{
					goToAngle(opt_angle);
					fprintf(stdout, "%f", opt_angle);
				}
				fprintf(stdout, " and rot ");
				if(opt_rot < MIN_ROT)
				{
					goToRot(MIN_ROT);
					fprintf(stdout, "%u", MIN_ROT);
				}			
				else if(opt_rot > MAX_ROT)
				{
					goToRot(MAX_ROT);
					fprintf(stdout, "%u", MAX_ROT);
				}
				else
				{
					goToRot(opt_rot);
					fprintf(stdout, "%f", opt_rot);
				}
				fprintf(stdout, "\r\n");
			}
			//Direct Moving
			else if (strcmp(cmd,"down") == 0)
			{
				goDown(valNum);
			}
			else if (strcmp(cmd,"up") == 0)
			{	
				goUp(valNum);
			}
			else if (strcmp(cmd,"left") == 0)
			{	
				goLeft(valNum);
			}
			else if (strcmp(cmd,"right") == 0)
			{
				goRight(valNum);
			}
			else if (strcmp(cmd,"accel") == 0)
			{	
				//reads accelerometer value
				updateAccel();
				dtostrf(curr_accel, 6, 3, accel_string);    
	    		fprintf(stdout, "%s\r\n", accel_string); 
			}
			else if (strcmp(cmd,"angle") == 0)
			{
				goToAngle(valNum);
			
			}
			else if (strcmp(cmd,"rot") == 0)
			{
				goToRot(valFloat);
			
			}
			else if (strcmp(cmd,"gps") == 0)
			{
				updateGPS();
			
			}
			//UTC Getter functions
			else if (strcmp(cmd,"time") == 0)
			{
				int localHour = UTC_hour + UTC_offset;
				if (localHour < 0)
				{
					localHour += 24;
				}
				fprintf(stdout, "Current time is %u:%u:%u\r\n", localHour, UTC_min, UTC_sec);
				formatTime();
				fprintf(stdout, "Formatted time string: %s\r\n", fmtTimeString);	
			}
			else if (strcmp(cmd,"date") == 0)
			{
				fprintf(stdout, "Current date %u %u, %u\r\n", UTC_month, UTC_day, UTC_year);
				formatDate();
				fprintf(stdout, "Formatted date string: %s\r\n", fmtDateString);	
			}
			else if (strcmp(cmd,"panel") == 0)
			{
				updatePanel();
				dtostrf(panelVoltage, 6, 3, v_string);
				fprintf(stdout, "panel voltage: %s\r\n", v_string);	
			}
			else if (strcmp(cmd,"record") == 0)
			{
				//print line for data logger
				printRecord();
			}
			//UTC Setter functions
			else if (strcmp(cmd,"sethour") == 0)
			{
				UTC_hour = valNum;
			}
			else if (strcmp(cmd,"setmin") == 0)
			{
				UTC_min = valNum;
			}
			else if (strcmp(cmd,"setsec") == 0)
			{
				UTC_sec = valNum;
			}
			else if (strcmp(cmd,"setday") == 0)
			{
				UTC_day = valNum;
			}
			else if (strcmp(cmd,"setmonth") == 0)
			{
				UTC_month = valNum;
			}
			else if (strcmp(cmd,"setyear") == 0)
			{
				UTC_year = valNum;
			}
			//Entering Auto Mode
			else if (strcmp(cmd,"auto") == 0)
			{
				autoMode = 1;
				fprintf(stdout, "entered auto mode\r\n");
				updateGPS();
				while(1)
				{
					if(autoFlag == 1)
					{
						fprintf(stdout, "gonna do stuff automatically\r\n");
						updateAccel();
						fprintf(stdout, "updated accel\r\n");
						updatePanel();
						fprintf(stdout, "updated panel\r\n");
						updateQuad();
						fprintf(stdout, "updated quad\r\n");
						optimize2();
						fprintf(stdout, "optimized\r\n");
						considerMoving();
						fprintf(stdout, "considered moving\r\n");
						autoFlag = 0;
					}
				}
				
			}
			//Enter Data Logger Mode
			else if (strcmp(cmd,"data") == 0)
			{
				dataLog = 1; //Set this flag first to avoid confusing Processing
				//Get updated values
				updatePanel();
				updateQuad();
				updateAccel();
				printRecord(); //Send a record to acknowledge command
				updateGPS();
				optimize2();
				autoMode = 1; //signal to ISR to update autoFlag
				
			}
			else
				fprintf(stdout, "What?\r\n");
		}
		else //dataLog == 1
		{	
			//In Data Logger mode.
			if(autoFlag == 1)
			{
				updateAccel();
				optimize2();
				updatePanel();
				updateQuad();
				printRecord();
				//fprintf(stdout, "so...\r\n");
				considerMoving();
				autoFlag = 0;
			}
		}
	}
	
}

//*****************************************************************************
// EOF
//*****************************************************************************
