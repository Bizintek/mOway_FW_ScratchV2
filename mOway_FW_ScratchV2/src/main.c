//******************************************************************************
//*                                Scratch RC                              	   *
//******************************************************************************
//*    Scratch RC                                                              *
//*    version: 5.0.1                                                          *
//*    Client for Radio Control the robot mOway by means of Scratch		       *
//*    Copyright (C) 2013  Bizintek Innova S.L.                                *
//******************************************************************************
//*    This program is free software; you can redistribute it and/or modify    *
//*    it under the terms of the GNU General Public License as published by    *
//*    the Free Software Foundation; either version 2 of the License, or       *
//*    (at your option) any later version.                                     *
//*                                                                            *
//*    This program is distributed in the hope that it will be useful,         *
//*    but WITHOUT ANY WARRANTY; without even the implied warranty of          *
//*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
//*    GNU General Public License for more details.                            *
//*                                                                            *
//*    You should have received a copy of the GNU General Public License along *
//*    with this program; if not, write to the Free Software Foundation, Inc., *
//*    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.             *
//******************************************************************************
//*    Date: 07/03/2013                                                        *
//******************************************************************************

//*****************************************************
//	INCLUDES                                          *
//*****************************************************
#include "p18f87j50.h"                                                          //Moway microcontroller

#include "lib_rf2gh4.h"                                                         //rf library

#include "lib_mot_moway.h"                                                      //Engines library

#include "lib_config_moway.h"                                                   //Sensor library
#include "lib_sen_led.h"
#include "lib_sen_obstac.h"
#include "lib_sen_light.h"
#include "lib_sen_speaker.h"
#include "lib_sen_micro.h"
#include "lib_sen_line.h"
#include "lib_sen_battery.h"
#include "lib_sen_accel.h"

//*****************************************************
//	DEFINITIONS                                       *
//*****************************************************
// Melodies
#define NO_MELODY		0
#define MELODY_CHARGE	1
#define MELODY_FAIL		2

// Programs
#define NO_PROGRAM		0
#define LINE_FOLLOW_L	1
#define LINE_FOLLOW_R	2
#define ENCLOSED		3
#define DEFENDER		4
#define PUSH			5

//Canal and direction of mOway
#define moway_address		0x02
#define pc_address			0x01

//Motion commands
#define cmd_go				0xE1
#define cmd_back			0xE2
#define cmd_go_left			0xE3
#define cmd_go_right		0xE4
#define cmd_back_left		0xE5
#define cmd_back_right		0xE6
#define cmd_stop			0xE7
#define cmd_rotate_left		0xE8
#define cmd_rotate_right	0xE9   	
#define cmd_go_simple		0xEA
#define cmd_back_simple		0xEB
#define cmd_left_simple		0xEC
#define cmd_right_simple	0xED
#define cmd_turn_around		0xEE
#define cmd_reset_dist		0xEF

//LED commands
#define cmd_led_front_on	0xA0
#define cmd_led_brake_on	0xA1
#define cmd_led_green_on	0xA2
#define cmd_led_red_on		0xA3
#define cmd_led_front_off	0xA4
#define cmd_led_brake_off	0xA5
#define cmd_led_green_off	0xA6
#define cmd_led_red_off		0xA7
#define cmd_led_front_blink	0xA8

//DEBUG
#define cmd_led_brake_blink	0xA9
#define cmd_led_green_blink	0xAA
#define cmd_led_red_blink	0xAB
#define cmd_leds_on			0xAC
#define cmd_leds_off		0xAD
#define cmd_leds_blink		0xAE

//Miscellaneous commands
#define cmd_var				0xB0
#define cmd_rst				0xB1
#define cmd_frame_01		0xB5
#define cmd_frame_02		0xB6

// Sound
#define cmd_buzzer_on		0xC0
#define cmd_buzzer_off		0xC1
#define cmd_melody_charge	0xC2
#define cmd_melody_fail		0xC3

// Functions
#define cmd_line_follow_l	0x91
#define cmd_line_follow_r	0x92
#define cmd_enclosed		0x93
#define cmd_defender		0x94
#define cmd_push			0x95
	
//*****************************************************
// BOOTLOADER JUMP                                    *
//*****************************************************
#define REMAPPED_RESET_VECTOR_ADDRESS			0x1000
#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x1008
#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x1018

//*****************************************************
//	              CONFIGURATION BITS                  *
//*****************************************************
#pragma config XINST    = OFF                                                   // Extended instruction set
#pragma config STVREN   = ON                                                    // Stack overflow reset
#pragma config PLLDIV   = 3                                                     // (12 MHz crystal used on this board)
#pragma config WDTEN    = ON                                                    // Watch Dog Timer (WDT)
#pragma config CP0      = OFF                                                   // Code protect
#pragma config CPUDIV   = OSC3_PLL3                                             // OSC1 = divide by 1 mode
#pragma config IESO     = OFF                                                   // Internal External (clock) Switchover
#pragma config FCMEN    = OFF                                                   // Fail Safe Clock Monitor
#pragma config FOSC     = HSPLL                                                 // Firmware must also set OSCTUNE<PLLEN> to start PLL!
#pragma config WDTPS    = 256                                                   // Aprox 1 sec
#pragma config MSSPMSK  = MSK5                                                  //
#pragma config CCP2MX   = ALTERNATE                                             //
#pragma config WAIT     = OFF                                                   // 
#pragma config BW       = 16                                                    // Only available on the
#pragma config MODE     = XM12                                                  // 80 pin devices in the 
#pragma config EASHFT   = OFF                                                   // family.
#pragma config PMPMX    = DEFAULT                                               //
#pragma config ECCPMX   = ALTERNATE                                             //
//******************************************************************************//
	
//*****************************************************
//	Variables                                         *
//*****************************************************
volatile unsigned char program;
volatile unsigned char melody;
volatile unsigned char melodyNote;
volatile unsigned char frontBlinks;
volatile unsigned char brakeBlinks;
volatile unsigned char greenBlinks;
volatile unsigned char redBlinks;
volatile unsigned char ledsBlinks;

// RF variables
char data_in[8];
char data_in_dir;

//Speed ??and initial radius
volatile unsigned char RC_VEL = 50;
unsigned char RC_RADIO = 20;
unsigned char RC_ROT = 20;
unsigned char RC_TIME = 100;
unsigned char RC_DISTANCE = 20;
unsigned char RC_WHEEL_CENTER = 0;
unsigned char RC_TIME_DISTANCE = 0;
unsigned char RC_TIME_DISTANCE_VAR = 0;
unsigned char RC_FREQ = 250;


#pragma romdata channel_section=0x1600
unsigned rom char channel  = 0x03;
#pragma romdata


//*****************************************************
//	Function definitions                              *
//*****************************************************
void Config();
void ConfigTimer(void);
void EnableTimer(void);
void DisableTimer();
void ResetTimer(void);
void ReadCommand();
void DelayTimer5(unsigned char times);
void LineFollow(char side);
void Enclosed(void);
void Defender(void);
void Push(void);
void MelodyCharge(unsigned char note);
void MelodyFail(unsigned char note);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();

// Function Config
void Config()
{
	RF_CONFIG_SPI();
	RF_OFF();
	RF_CONFIG(channel,moway_address);
	RF_INT_EN();
	RF_ON();
	SEN_CONFIG();
	MOT_CONFIG();
}	

//Function ReadCommand
void ReadCommand ()
{	
	RC_VEL=data_in[1];
	RC_RADIO=data_in[6];
	RC_ROT=data_in[2];
	RC_TIME=data_in[3];
	RC_DISTANCE=data_in[4];
	RC_WHEEL_CENTER =data_in[5];
	RC_FREQ=data_in[7];
	
	// Engine
	if ((data_in[0]>>4)==0x0E)                                                  //Movement command
	{
		// Stop the current program (all affect movement)
		program = NO_PROGRAM;			
		
		if (RC_TIME!=0)
		{
			RC_TIME_DISTANCE=TIME;
			RC_TIME_DISTANCE_VAR=RC_TIME;
		}
		else if (RC_DISTANCE!=0)
		{
			RC_TIME_DISTANCE=DISTANCE;
			RC_TIME_DISTANCE_VAR=RC_DISTANCE;
		}
		else
		{		
			RC_TIME_DISTANCE=TIME;
			RC_TIME_DISTANCE_VAR=0;
		}
		if (RC_RADIO + RC_VEL >100)
			RC_RADIO = 100 - RC_VEL;
	}
	
	// Movement
	if(data_in[0]==cmd_go)                                                      //Forward
		MOT_STR(RC_VEL,FWD,RC_TIME_DISTANCE,RC_TIME_DISTANCE_VAR);
	else if(data_in[0]==cmd_back)                                               //Back
		MOT_STR(RC_VEL,BACK,RC_TIME_DISTANCE,RC_TIME_DISTANCE_VAR);
	else if(data_in[0]==cmd_go_left)                                            //Forward-left
		MOT_CUR(RC_VEL,FWD,RC_RADIO,LEFT,RC_TIME_DISTANCE,RC_TIME_DISTANCE_VAR);
	else if(data_in[0]==cmd_go_right)                                           //Forward-right
		MOT_CUR(RC_VEL,FWD,RC_RADIO,RIGHT,RC_TIME_DISTANCE,RC_TIME_DISTANCE_VAR);
	else if(data_in[0]==cmd_back_left)                                          //Back-left
		MOT_CUR(RC_VEL,BACK,RC_RADIO,LEFT,RC_TIME_DISTANCE,RC_TIME_DISTANCE_VAR);
	else if(data_in[0]==cmd_back_right)                                         //Back-right
		MOT_CUR(RC_VEL,BACK,RC_RADIO,RIGHT,RC_TIME_DISTANCE,RC_TIME_DISTANCE_VAR);
	else if(data_in[0]==cmd_stop)                                               //Stop
		MOT_STOP();
	else if (data_in[0]==cmd_rotate_right)                                      // Right rotation
		MOT_ROT(RC_VEL, FWD, RC_WHEEL_CENTER, RIGHT, ANGLE, RC_ROT) ;
	else if (data_in[0]==cmd_rotate_left)                                       // Left rotation
		MOT_ROT(RC_VEL, FWD, RC_WHEEL_CENTER, LEFT, ANGLE, RC_ROT) ;
	else if (data_in[0] == cmd_go_simple)                                       // Advance to 50% indefinitely
		MOT_STR(50, FWD, TIME, 0);
	else if (data_in[0] == cmd_back_simple)                                     // Go back to 50% indefinitely
		MOT_STR(50, BACK, TIME, 0);
	else if (data_in[0] == cmd_left_simple)                                     // Turn 90 degrees to the left
		MOT_ROT(50, FWD, RC_WHEEL_CENTER, LEFT, ANGLE, 25);
	else if (data_in[0] == cmd_right_simple)                                    // Turn 90 degrees to the right
		MOT_ROT(50, FWD, RC_WHEEL_CENTER, RIGHT, ANGLE, 25);
	else if (data_in[0] == cmd_turn_around)                                     //Turn around
		MOT_ROT(50, FWD, RC_WHEEL_CENTER, RIGHT, ANGLE, 50);	
	else if(data_in[0] == cmd_reset_dist)                                       //Reset speedometer
		MOT_RST(RST_KM);	
	
	//LED
	else if(data_in[0] == cmd_led_front_on)                                     //Left Led Change
		LED_FRONT_ON();
	else if(data_in[0] == cmd_led_brake_on)                                     //Right Led Change
		LED_BRAKE_ON();		
	else if(data_in[0] == cmd_led_green_on)                                     //Green Led Change
		LED_TOP_GREEN_ON();
	else if(data_in[0] == cmd_led_red_on)                                       //Red Led Change
		LED_TOP_RED_ON();
	else if(data_in[0] == cmd_led_front_off)                                    //Left Led Change
		LED_FRONT_OFF();
	else if(data_in[0] == cmd_led_brake_off)                                    //Right Led Change
		LED_BRAKE_OFF();
	else if(data_in[0] == cmd_led_green_off)                                    //Green Led Change
		LED_TOP_GREEN_OFF();
	else if(data_in[0] == cmd_led_red_off)                                      //Red Led Change
		LED_TOP_RED_OFF();	
		
	else if(data_in[0] == cmd_led_front_blink)
	{			
		if(frontBlinks < 5)
			frontBlinks++;			
	}
    
	else if(data_in[0] == cmd_led_brake_blink)
	{			
		if(brakeBlinks < 5)
			brakeBlinks++;				
	}
    
	else if(data_in[0] == cmd_led_green_blink)
	{
		if(greenBlinks < 5)
			greenBlinks++;		
	}
    
	else if(data_in[0] == cmd_led_red_blink)
	{	
		if(redBlinks < 5)
			redBlinks++;		
	}
		
	else if(data_in[0] == cmd_leds_on)
	{
		LED_FRONT_ON();
		LED_BRAKE_ON();
		LED_TOP_GREEN_ON();
		LED_TOP_RED_ON();
	}
    
	else if(data_in[0] == cmd_leds_off)
	{
		LED_FRONT_OFF();
		LED_BRAKE_OFF();
		LED_TOP_GREEN_OFF();
		LED_TOP_RED_OFF();
	}
    
	else if(data_in[0] == cmd_leds_blink)
	{
		if(ledsBlinks < 5)
			ledsBlinks++;			
	}				
		
	// Sound	
	else if(data_in[0] == cmd_buzzer_on)
		SEN_SPEAKER(RC_FREQ, 1, SPEAKER_ON);
	else if(data_in[0] == cmd_buzzer_off)
		SEN_SPEAKER(RC_FREQ, 1, SPEAKER_OFF);
		
	else if(data_in[0] == cmd_melody_charge)
		melody = MELODY_CHARGE;		
	
	else if(data_in[0] == cmd_melody_fail)	
		melody = MELODY_FAIL;				


	else if(data_in[0] == cmd_rst)                                              //Reset
	{
		RC_VEL = 50;
		RC_RADIO = 20;
		RC_ROT = 20;
		RC_TIME = 100;
		RC_DISTANCE = 20;
		RC_WHEEL_CENTER = 0;
		RC_TIME_DISTANCE = 0;
		LED_FRONT_OFF();
		LED_BRAKE_OFF();
		LED_TOP_RED_OFF();
		LED_TOP_GREEN_OFF();
	}	
	
	//*************************************************
	//	                 Programs                     *
	//*************************************************		
	else if(data_in[0] == cmd_line_follow_l)	
		program = LINE_FOLLOW_L;
	else if(data_in[0] == cmd_line_follow_r)	
		program = LINE_FOLLOW_R;		
	else if(data_in[0] == cmd_enclosed)	
		program = ENCLOSED;		
	else if(data_in[0] == cmd_defender)	
		program = DEFENDER;
	else if(data_in[0] == cmd_push)	
		program = PUSH;	
}  

extern void _startup (void);     
#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
void _reset (void)
{
    _asm goto _startup _endasm
}

#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS

void Remapped_High_ISR (void)
{
    _asm goto YourHighPriorityISRCode _endasm
}

#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS

void Remapped_Low_ISR (void)
{
    _asm goto YourLowPriorityISRCode _endasm
}

#pragma code

#pragma interrupt YourHighPriorityISRCode

//Function YourHighPriorityISRCode
void YourHighPriorityISRCode()	
{			
	
    // Disable global interrupts. Interrutps are enabled at ISR exit.
	INTCONbits.GIEH = 0;
	
	if(INTCONbits.TMR0IF == 1)
	{	
		INTCONbits.TMR0IF = 0;
		_asm reset _endasm		
	}

	else if(INTCONbits.INT0IF == 1)
	{
		INTCONbits.INT0IF = 0;
		INTCONbits.TMR0IF = 0;		
		
		while(RF_RECEIVE((unsigned char*)&data_in_dir,(unsigned char*)&data_in)!=2)		
			ReadCommand();													
	}	
}	

#pragma interruptlow YourLowPriorityISRCode

//Function YourLowPriorityISRCode
void YourLowPriorityISRCode()	
{

}	
	          
//********************DECLARATIONS**********************//
#pragma code
//**********************[MAIN]*************************//
void main()
{   
	
    char iMic;
	unsigned char micValue1 = 0;
	unsigned char micValue2 = 0; 
	static char data_out[8];	
	  
	// Initialize melody and LEDs controllers
	melody = NO_MELODY;
	melodyNote = 0;	
	frontBlinks = 0;
	brakeBlinks = 0;
	greenBlinks = 0;	
	redBlinks = 0;
	ledsBlinks = 0;
	
	// Initialize hardware and Timer 0
	Config();
    
	//DEBUG	
    ConfigTimer();	
	EnableTimer();	
	
	while(1)
	{		
        ResetTimer();	
		
		//*********************************************
		//	     Average microphone values            *
		//*********************************************
		micValue2 = 0;
		for(iMic=0; iMic<5; iMic++)
		{
			micValue1 = SEN_MIC_ANALOG();
			if(micValue1 > micValue2)
				micValue2 = micValue1;
			Delay1KTCYx(10);                                                    // Delay 10ms
			
			//***********************************************************
			//	Programs: Line follow works better if called every 10ms *
			//***********************************************************	
			if(program == LINE_FOLLOW_L)
			{
				INTCONbits.GIEH = 0;                                            // Disable global interrupts
				LineFollow(LINE_FOLLOW_L);
				INTCONbits.GIEH = 1;                                            // Enable global interrupts
			}			
			else if(program == LINE_FOLLOW_R)
			{
				INTCONbits.GIEH = 0;                                            // Disable global interrupts
				LineFollow(LINE_FOLLOW_R);
				INTCONbits.GIEH = 1;                                            // Enable global interrupts
			}			
		}
									
		//*********************************************
		//	Send sensor data                          *
		//*********************************************
		// Obstacle sensors
        data_out[0] = SEN_OBS_ANALOG(OBS_SIDE_L);
        data_out[1] = SEN_OBS_ANALOG(OBS_CENTER_L);
		data_out[2] = SEN_OBS_ANALOG(OBS_CENTER_R);
        data_out[3] = SEN_OBS_ANALOG(OBS_SIDE_R);

        // Line sensors
        data_out[4] = SEN_LINE_ANALOG(LINE_R);
        data_out[5] = SEN_LINE_ANALOG(LINE_L);

        // Light sensors
        data_out[6] = MOT_END;
		
		// Frame number
		data_out[7] = cmd_frame_01;
		
		// Send critical sensor values;		
		RF_SEND(pc_address,(unsigned char*)&data_out);
		
		ResetTimer();
		
		// Accelerometer
		data_out[0] = SEN_ACCE_XYZ_READ(ACCE_CHECK_X);
		data_out[1] = SEN_ACCE_XYZ_READ(ACCE_CHECK_Y);
		data_out[2] = SEN_ACCE_XYZ_READ(ACCE_CHECK_Z);

		// Distance
		data_out[3] = MOT_FDBCK(STATUS_KM)[1];
		data_out[4] = MOT_FDBCK(STATUS_KM)[0];
		
		// Light sensor
		data_out[5] = SEN_LIGHT();
		
		// Microphone
		data_out[6] = micValue2;
		
		// Frame number
		data_out[7] = cmd_frame_02;
	
		// Send less critical sensor values		
		RF_SEND(pc_address,(unsigned char*)&data_out);
		
		ResetTimer();		
		
		//*********************************************
		//	LEDs blinking                             *
		//*********************************************			
		if(frontBlinks >= 5)
			frontBlinks = 0;
		if(brakeBlinks >= 5)
			brakeBlinks = 0;
		if(greenBlinks >= 5)
			greenBlinks = 0;
		if(redBlinks >= 5)
			redBlinks = 0;
		if(ledsBlinks >= 5)
			ledsBlinks = 0;			
		if(frontBlinks > 0)
		{
			INTCONbits.GIEH = 0;                                                // Disable global interrupts
			ResetTimer();			
			LED_FRONT_ON_OFF();			
			frontBlinks--;
			INTCONbits.GIEH = 1;                                                // Enable global interrupts			
		}	
		if(brakeBlinks > 0)
		{			
			INTCONbits.GIEH = 0;                                                // Disable global interrupts
			ResetTimer();							
			//DEBUG
			LED_BRAKE_ON_OFF();			
			LED_BRAKE_OFF();
			brakeBlinks--;			
			INTCONbits.GIEH = 1;                                                // Enable global interrupts	
		}			
		if(greenBlinks > 0)
		{			
			INTCONbits.GIEH = 0;                                                // Disable global interrupts
			ResetTimer();			
			LED_TOP_GREEN_ON_OFF();			
			greenBlinks--;
			INTCONbits.GIEH = 1;                                                // Enable global interrupts	
		}		
		if(redBlinks > 0)
		{			
			INTCONbits.GIEH = 0;                                                // Disable global interrupts
			ResetTimer();			
			LED_TOP_RED_ON_OFF();			
			redBlinks--;
			INTCONbits.GIEH = 1;                                                // Enable global interrupts	
		}
		if(ledsBlinks > 0)
		{			
			INTCONbits.GIEH = 0;                                                // Disable global interrupts
			ResetTimer();	
			LED_FRONT_ON();
			LED_BRAKE_ON();
			LED_TOP_GREEN_ON();
			LED_TOP_RED_ON();
			Delay10KTCYx(5);
			LED_FRONT_OFF();
			LED_BRAKE_OFF();
			LED_TOP_GREEN_OFF();
			LED_TOP_RED_OFF();			
			ledsBlinks--;
			INTCONbits.GIEH = 1;                                                // Enable global interrupts	
		}
				
		//*********************************************
		//	Melodies                                  *
		//*********************************************
		if(melody == MELODY_CHARGE)
		{			
			INTCONbits.GIEH = 0;                                                // Disable global interrupts
			if(melodyNote > 6)
			{
				melody = NO_MELODY;
				melodyNote = 0;
			}
			else
			{
				MelodyCharge(melodyNote);
				melodyNote++;
			}
			INTCONbits.GIEH = 1;                                                // Enable global interrupts
		}
		else if(melody == MELODY_FAIL)
		{
			INTCONbits.GIEH = 0;                                                // Disable global interrupts
			if(melodyNote > 3)
			{
				melody = NO_MELODY;
				melodyNote = 0;
			}
			else
			{
				MelodyFail(melodyNote);
				melodyNote++;
			}
			INTCONbits.GIEH = 1;                                                // Enable global interrupts
		}

		//***********************************************************
		//	Programs: Line follow works better if called every 10ms *
		//***********************************************************	
		if(program == LINE_FOLLOW_L)
		{ 	}			
		else if(program == LINE_FOLLOW_R)
		{	}
		else if(program == ENCLOSED)
		{
			INTCONbits.GIEH = 0;                                                // Disable global interrupts
			Enclosed();
			INTCONbits.GIEH = 1;                                                // Enable global interrupts
		}			
		else if(program == DEFENDER)
		{
			INTCONbits.GIEH = 0;                                                // Disable global interrupts
			Defender();						
			INTCONbits.GIEH = 1;                                                // Enable global interrupts
		}			
		else if(program == PUSH)
		{
			INTCONbits.GIEH = 0;                                                // Disable global interrupts
			Push();	
			INTCONbits.GIEH = 1;                                                // Enable global interrupts
		}		
	}
}

//*****************************************************
//	                FUNCTIONS                         *
//*****************************************************

//Function ConfigTimer
void ConfigTimer()
{	
	TMR0H = 0;
	TMR0L = 0;	
	INTCONbits.TMR0IF = 0;                                                      // Must be cleared in software
	T0CONbits.T08BIT = 0;                                                       // 16 bit timer
	T0CONbits.T0CS = 0;                                                         // Internal instruction cycle clock
	T0CONbits.PSA = 0;                                                          // Prescaler
	T0CONbits.T0PS2 = 0;                                                        // Prescaler -> 1:2 -> 250ms
	T0CONbits.T0PS1 = 0;		
	T0CONbits.T0PS0 = 0;
                                                                                // Enables timer 0
	
    INTCON2bits.TMR0IP = 1;                                                     // High priority
	INTCONbits.TMR0IE = 0;                                                      // Disable interrupt
	INTCONbits.GIEH = 1;
                                                                                // Disable low level interrupts	
}

//Function EnableTimer
void EnableTimer()
{	
	T0CONbits.TMR0ON = 1;                                                       // Enable timer 0
	INTCONbits.TMR0IE = 1;                                                      // Enable interrupt	
}

//Function DisableTimer
void DisableTimer()
{	
	T0CONbits.TMR0ON = 0;                                                       // Disables timer 0
	INTCONbits.TMR0IE = 0;                                                      // Disables interrupt	
}

//Function ResetTimer
void ResetTimer()
{
	TMR0H = 0;
	TMR0L = 0;
}

//Function DelayTimer5
void DelayTimer5(unsigned char times)
{
	char i;
	
	TMR0H = 0;
	TMR0L = 0;
	
	for(i=0; i<times; i++)
	{		
		Delay10KTCYx(5);
		TMR0H = 0;
		TMR0L = 0;
	}	
}

//*****************************************************
//	                   Melodies                       *
//*****************************************************	

//Function MelodyCharge
void MelodyCharge(unsigned char note)
{
	ResetTimer();
	
	switch(note)
	{
		case 0:
			SEN_SPEAKER(80, 2, SPEAKER_ON);                                     // Sol
			Delay10KTCYx(10);
			ResetTimer();			
			break;
		case 1:
			SEN_SPEAKER(71, 2, SPEAKER_ON);                                     // La
			Delay10KTCYx(10);
			ResetTimer();			
			break;
		case 2:
			SEN_SPEAKER(62, 2, SPEAKER_ON);                                     // Si
			Delay10KTCYx(10);
			ResetTimer();			
			break;
		case 3:
			SEN_SPEAKER(59, 3, SPEAKER_ON);                                     // Do
			Delay10KTCYx(10);
			ResetTimer();
			Delay10KTCYx(10);
			ResetTimer();			
			break;		
		case 4:
			SEN_SPEAKER(80, 2, SPEAKER_ON);                                     // Sol
			Delay10KTCYx(10);
			ResetTimer();			
			break;			
		case 6:
			SEN_SPEAKER(59, 3, SPEAKER_ON);                                     // Do
			Delay10KTCYx(10);
			ResetTimer();
			Delay10KTCYx(10);
			ResetTimer();
			SEN_SPEAKER(93, 1, SPEAKER_OFF);
			break;		
	}								
}

//Function MelodyFail
void MelodyFail(unsigned char note)
{
	ResetTimer();
	
	switch(note)
	{
		case 0:
			SEN_SPEAKER(93, 2, SPEAKER_ON);                                     // Mi
			Delay10KTCYx(10);
			ResetTimer();
			break;
		case 1:
			SEN_SPEAKER(99, 2, SPEAKER_ON);                                     // Re#
			Delay10KTCYx(10);
			ResetTimer();
			break;
		case 2:
			SEN_SPEAKER(105, 2, SPEAKER_ON);                                    // Re
			Delay10KTCYx(10);
			ResetTimer();
			break;
		case 3:
			SEN_SPEAKER(111, 3, SPEAKER_ON);                                    // Do#
			Delay10KTCYx(10);   
			ResetTimer();
			Delay10KTCYx(10);
			ResetTimer();			
			SEN_SPEAKER(93, 1, SPEAKER_OFF);
			break;	
	}
}

//*****************************************************
//	                   Programs                       *
//*****************************************************	

//*****************************************************
//	    Follow a line (right side or left side)       *
//*****************************************************

//Function LineFollow
void LineFollow(char side)
{	
	unsigned char speed = RC_VEL;
	
	if(side == LINE_FOLLOW_L)
	{
		// Over the line
		if(SEN_LINE_DIG(LINE_L) == 0 && SEN_LINE_DIG(LINE_R) == 1)
			MOT_STR(speed, FWD, TIME, 0);	

		// Turn right
		else if(SEN_LINE_DIG(LINE_L) == 0 && SEN_LINE_DIG(LINE_R) == 0)	
			MOT_ROT(speed, FWD, WHEEL, RIGHT, TIME, 0);	

		// Turn left
		else if(SEN_LINE_DIG(LINE_L) == 1 && SEN_LINE_DIG(LINE_R) == 1)	
			MOT_ROT(speed, FWD, WHEEL, LEFT, TIME, 0);	

		// Turn left
		else if(SEN_LINE_DIG(LINE_L) == 1 && SEN_LINE_DIG(LINE_R) == 0)	
			MOT_ROT(speed, FWD, WHEEL, LEFT, TIME, 0);	
	}
	else if(side == LINE_FOLLOW_R)
	{
		// Over the line
		if(SEN_LINE_DIG(LINE_L) == 1 && SEN_LINE_DIG(LINE_R) == 0)
			MOT_STR(speed, FWD, TIME, 0);	

		// Turn right
		else if(SEN_LINE_DIG(LINE_L) == 1 && SEN_LINE_DIG(LINE_R) == 1)				
			MOT_ROT(speed, FWD, WHEEL, RIGHT, TIME, 0);	

		// Turn left
		else if(SEN_LINE_DIG(LINE_L) == 0 && SEN_LINE_DIG(LINE_R) == 0)				
			MOT_ROT(speed, FWD, WHEEL, LEFT, TIME, 0);	

		// Turn right
		else if(SEN_LINE_DIG(LINE_L) == 0 && SEN_LINE_DIG(LINE_R) == 1)				
			MOT_ROT(speed, FWD, WHEEL, RIGHT, TIME, 0);	
	}	
}

//*****************************************************
//	         Enclosed into a black circle             *
//*****************************************************

//Function Enclosed
void Enclosed()
{
	unsigned char speed = RC_VEL;
	unsigned char distance = 10;	
	
	MOT_STR(speed, FWD, TIME, 0);
	
	if(SEN_LINE_DIG(LINE_L) == 1 || SEN_LINE_DIG(LINE_R) == 1)
	{
		// Rotate 144º
		MOT_ROT(speed, FWD, CENTER, RIGHT, ANGLE, 40);
		DelayTimer5(14);                                                        // Delay 700ms (resets Timer 0)
	}	
}

//***********************************************************
//	Detects obstacles an pushes them out of a black circle  *
//***********************************************************

//Function Defender
void Defender()
{
	unsigned char speed = RC_VEL;
	unsigned char distance = 10;
		
	MOT_STR(speed, FWD, TIME, 0);
	
	if(SEN_LINE_DIG(LINE_L) == 1 || SEN_LINE_DIG(LINE_R) == 1)
	{
		// Rotate 144º
		MOT_ROT(speed, FWD, CENTER, RIGHT, ANGLE, 40);
		DelayTimer5(14);                                                        // Delay 700ms (resets Timer 0)
	}
	else if(SEN_OBS_ANALOG(OBS_CENTER_R) > distance || SEN_OBS_ANALOG(OBS_CENTER_R) > distance)
		return;	
	else if(SEN_OBS_ANALOG(OBS_SIDE_R) > distance)
	{
		// Rotate 14.4º
		MOT_ROT(speed, FWD, CENTER, RIGHT, ANGLE, 4);		
		DelayTimer5(4);                                                         // Delay 200ms (resets Timer 0)
	}
	else if(SEN_OBS_ANALOG(OBS_SIDE_L) > distance)
	{
		// Rotate 14.4º
		MOT_ROT(speed, FWD, CENTER, LEFT, ANGLE, 4);		
		DelayTimer5(4);                                                         // Delay 200ms (resets Timer 0)
	}		
}

//**********************************************
//	    Detects an obstacle an pushes it       *
//**********************************************

//Function Push
void Push()
{
	unsigned char speed = RC_VEL;
	unsigned char distance = 50;
		
	MOT_STR(speed, FWD, TIME, 0);	
	
	if(SEN_OBS_ANALOG(OBS_CENTER_R) > distance || SEN_OBS_ANALOG(OBS_CENTER_R) > distance)
		return;	
	
	else if(SEN_OBS_ANALOG(OBS_SIDE_R) > distance)
	{
		// Rotate 14.4º
		MOT_ROT(speed, FWD, CENTER, RIGHT, ANGLE, 4);
		DelayTimer5(4);                                                         // Delay 200ms (resets Timer 0)
	}
	else if(SEN_OBS_ANALOG(OBS_SIDE_L) > distance)
	{
		// Rotate 14.4º
		MOT_ROT(speed, FWD, CENTER, LEFT, ANGLE, 4);
		DelayTimer5(4);                                                         // Delay 200ms (resets Timer 0)
	}	
}

	


	