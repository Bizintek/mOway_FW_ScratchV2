//******************************************************************************
//*    lib_rf2gh4.c                                                            *
//*    version: 5.0.1                                                          *
//*    This library contains the necessary functions to manage the             *
//*    module RF2GH4 with the C18 compiler.                                    *
//*    Copyright (C) 2010  Bizintek Innova S.L.                                *
//*                                                                            *
//******************************************************************************
//*    This program is free software; you can redistribute it and/or modify    *
//*    it under the terms of the GNU General Public License as published by    *
//*    the Free Software Foundation; either version 3 of the License, or       *
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

#include "lib_rf2gh4.h"

//**********************************************
//*                 VARIABLES                  *
//**********************************************

//Internal variables
static unsigned char  	   DATA_N_SND;
static unsigned char        DATA_N_RCV;

//**********************************************
//*                   CODE                     *
//**********************************************

//Internal Functions
unsigned char spi_read(unsigned char);
void spi_write(unsigned char);

//*****************************************************


//*****************************************************
//*               RF_CONFIG_SPI()                     *
//*****************************************************
//*Description: The function configures the SPI module*
//*microcontroller. It is specified as output SDO     *
//*and as SDI input among other parameters of SPI     *
//*protocol.                                          *
//*****************************************************
//*Input variables:                                   *
//*Output variables:                                  *
//*****************************************************
void RF_CONFIG_SPI(void)
{
   //Configuration I/O.
   SCK_TRIS=0;
   SDI_TRIS=1;
   SDO_TRIS=0;
   RF_CS_TRIS=0;
   RF_CE_TRIS=0;
   RF_IRQ_TRIS=1;

	//Configuration of the SPI module.
	SSP1STAT=0b11000000;
	SSPCON1=0b00100000;
	SSPCON2=0b00000000;
}

//*****************************************************



//*****************************************************
//*                    RF_INT_EN()                    *
//*****************************************************
//*Description: It is responsible for enabling the    *
//*interruption external (RB0) used by the RF module  *
//*in the data reception.                             *
//*****************************************************
//*Input variables:                                   *
//*Output variables:                                  *
//*****************************************************

void RF_INT_EN(void)
{

    //Enable external interrupts with edge descent.
   	INTCONbits.GIE=0;
   	INTCON2bits.INTEDG0=0;
	RF_IRQ=1;
   	INTCONbits.INT0E=1;
	INTCONbits.PEIE=1;
	INTCONbits.GIE=1;
}

//*****************************************************



//*****************************************************
//* RF_CONFIG(unsigned char canal, unsigned char dir) *
//*****************************************************
//*Description:This function is responsible for       *
//*configuring the transceiver enabling its own       *
//*address listen and the channel among other         *
//*parameters.                                        *
//*****************************************************
//*Input variables:     - Channel                     *
//*                     - Address                     *
//*Output variables:    - 0: Not configured           *
//*                     - 1: Correct configuration    *
//*****************************************************

unsigned char RF_CONFIG(unsigned char canal, unsigned char dir)
{
	static unsigned char CONFIG_TEST;	
	static unsigned char INTER_RF;
	
	if(INTCONbits.GIE)
      INTER_RF = 1;
   else
      INTER_RF = 0;

   INTCONbits.GIE = 0;
	
   //Configuration I/O.
   SCK_TRIS=0;
   SDI_TRIS=1;
   SDO_TRIS=0;
   RF_CS_TRIS=0;
   RF_CE_TRIS=0;
   RF_IRQ_TRIS=1;
   RF_CE=0;

   //Configuration of the random shipping address.
   //In the send function, the address is configured desired by the user.
   RF_CS=0;
   spi_write(0x30);
   spi_write(0xFF);
   spi_write(0xC2);
   spi_write(0xC2);
   spi_write(0xC2);
   spi_write(0xC2);
   RF_CS=1;

   //Configuration of the Pipe0 address for ACK reception.
   
   RF_CS=0;
   spi_write(0x2A);
   spi_write(0xFF);
   spi_write(0xC2);
   spi_write(0xC2);
   spi_write(0xC2);
   spi_write(0xC2);
   RF_CS=1;

   // RX_ADDR_P1 (dir)
   //Setting the address of Pipe1 for the recpetion of frames.
   RF_CS=0;
   spi_write(0x2B);
   spi_write(dir);
   spi_write(0xC2);
   spi_write(0xC2);
   spi_write(0xC2);
   spi_write(0xC2);
   RF_CS=1;

   // RX_ADDR_P2 (0x00) BROADCAST
   //Setting the address of Pipe2 for the recpetion of frames.
   RF_CS=0;
   spi_write(0x2C);
   spi_write(0x00);
   RF_CS=1;


   // EN_AA
   // Enable AutoAck on Pipe0, Pipe1 and Pipe2.
   RF_CS=0;
   spi_write(0x21);
   spi_write(0x07);
   RF_CS=1;

   // EN_RXADDR
   //Enable Pipe0, Pipe1 and Pipe2.
   RF_CS=0;
   spi_write(0x22);
   spi_write(0x07);
   RF_CS=1;

   // SETUP_AW
   //Configuration of the length of the addresses. 5-byte addresses.
   RF_CS=0;
   spi_write(0x23);
   spi_write(0x03);
   RF_CS=1;

   //SETUP_RETR
   //Configuration of transmisssion retransmissions.
   //Ten retransmissions every 336 us. 
  
   RF_CS=0;
   spi_write(0x24);
   spi_write(0x0A);
   RF_CS=1;

   //RF_SETUP
   //RF aspects configuration.
   //Maximum LNA gain, 0dBm output power and 2 MBps speed.
   RF_CS=0;
   spi_write(0x26);
   spi_write(0x0F);
   RF_CS=1;

   //STATUS 
   // Resetting the STATUS register
   RF_CS=0;
   spi_write(0x27);
   spi_write(0x70);
   RF_CS=1;

   //RX_PW_P0
   //Nº of bytes in Pipe0.
   //1 byte (ACK).
   RF_CS=0;
   spi_write(0x31);
   spi_write(0x01);
   RF_CS=1;

   //RX_PW_P1
   //Nº of bytes in Pipe1.
   //10 byte (Sender address and frame).
   RF_CS=0;
   spi_write(0x32);
   spi_write(0x0A);
   RF_CS=1;

   //RX_PW_P2
   //Nº of bytes in Pipe2.
   //10 byte (Sender address and frame).
   RF_CS=0;
   spi_write(0x33);
   spi_write(0x0A);
   RF_CS=1;

   //RF_CH
   //Channel configuration
   //Channel chosen by the user(0x01 - 0x7F).
   RF_CS=0;
   spi_write(0x25);
   spi_write(canal);
   RF_CS=1;

   // RF_CH
   //We read RF_CH yo verify that the module has been configured correctly.
   RF_CS=0;
   spi_write(0x05);
   CONFIG_TEST=spi_read(0);
   RF_CS=1;

   if(CONFIG_TEST!=canal)
   {
		INTCONbits.GIE = INTER_RF;
		return(0);
	}
   else
   {
		INTCONbits.GIE = INTER_RF;
      return(1);
	}
}
//*****************************************************



//*****************************************************
//*                    RF_ON()                        *
//*****************************************************
//*Description: This routine activates the module     *
//*radiofrequency in listening mode to be able to     *
//*receive data sent to your address.                 *
//*****************************************************
//*Input variables:                                   *
//*Output variables:                                  *
//*Output:              - 0: RF not connected         *
//*                     - 1: Correct connection       *
//*****************************************************
unsigned char RF_ON(void)
{
	static unsigned char CONFIG_TEST;
	static unsigned char INTER_RF;
	
	
	if(INTCONbits.GIE)
      INTER_RF=1;
   else
      INTER_RF=0;

   INTCONbits.GIE=0;
   
   RF_CE=0;

   // CONFIG
   //The module is activated, it is placed in reception, 
   //the CRC is activated to use 2 bytes.
   RF_CS=0;
   spi_write(0x20);
   spi_write(0x0F);
   RF_CS=1;

   // CONFIG
   //We read config to chech that the module has been activated correctly.
 
   RF_CS=0;
   spi_write(0x00);
   CONFIG_TEST=spi_read(0);
   RF_CS=1;

   Delay10TCYx (200);                                                           //Delay 2ms
   RF_CE=1;
   Delay10TCYx (15);                                                            //Delay 150us
   
   INTCONbits.INT0F = 0;   

   if(CONFIG_TEST!=0x0F)
	{
		INTCONbits.GIE = INTER_RF;
		return(0);
	}
   else
	{
		INTCONbits.GIE = INTER_RF;
		return(1);
	}
}

//*****************************************************



//*****************************************************
//*                 RF_OFF()                          *
//*****************************************************
//*Description: This procedure desactivates the       *
//*module of radiofrecuency                           *
//*****************************************************
//*Input variables:                                   *
//*Output variables:                                  *
//*Output:              - 0: RF not connected         *
//*                     - 1: Correct connection       *
//*****************************************************
unsigned char RF_OFF(void)
{
	static unsigned char CONFIG_TEST;
	static unsigned char INTER_RF;
	static int dummy=0;
	
	if(INTCONbits.GIE)
      INTER_RF=1;
   else
      INTER_RF=0;

   INTCONbits.GIE=0;
   
   RF_CE=0;

   // CONFIG
   //The module is deactivated.
   RF_CS=0;
  
   spi_write(0x20);
   spi_write(0x0C);
   RF_CS=1;

	dummy++;
   // CONFIG
   //We read config to chech that the module has been deactivated correctly.
   RF_CS=0;
   spi_write(0x00);
   CONFIG_TEST=spi_read(0);
   RF_CS=1;
   
   INTCONbits.INT0F=0;
		  
   if(CONFIG_TEST!=0x0C)
	{
		INTCONbits.GIE = INTER_RF;
		return(0);
	}
   else
	{
		INTCONbits.GIE = INTER_RF;
		return(1);
	}
}

//*****************************************************



//*****************************************************
//*                 RF_SEND()                         *
//*****************************************************
//*Description: This function sends 8 bytes of data to*
//*the indicated address informing of the correct one *
//*reception in the recipient                         *
//*****************************************************
//*Input variables:      - RF_DATA_OUT[]              *
//                       - RF_DIR_OUT                 * 
//*Output variables:                                  *
//*Output:               - 0: Correct sending (ACK OK)*
//*                      - 1: Not received (NO ACK)   *
//*                      -2:Not sent                  *
//*****************************************************

unsigned char RF_SEND(unsigned char RF_DIR_OUT, unsigned char RF_DATA_OUT[])
{
   unsigned char i;
   unsigned char estado;
   static unsigned char RF_PAY_OUT[8];
   static unsigned char RF_PAY_DIR;
   static unsigned char INTER_RF;
   static int NO_RF;

	
   if(INTCONbits.GIE)
      INTER_RF=1;
   else
      INTER_RF=0;

   INTCONbits.GIE=0;

   // INICIO
   RF_CE=0;

   //STATUS
   //Resetting the STATUS register
   RF_CS=0;
   spi_write(0x27);
   spi_write(0x70);
   RF_CS=1;

   // EN_RXADDR
   //Pipe0 is enabled for ACK reception
   RF_CS=0;
   spi_write(0x22);
   spi_write(0x01);
   RF_CS=1;

   // TX_ADDR
   //The transmission address is set = RF_DIR
   RF_CS=0;
   spi_write(0x30);
   spi_write(RF_DIR_OUT);
   spi_write(0xC2);
   spi_write(0xC2);
   spi_write(0xC2);
   spi_write(0xC2);
   RF_CS=1;

   // RX_ADDR_P0
   //To receive the ACK, Pipe0 must be configured with 
   //the same address to transmit.
   RF_CS=0;
   spi_write(0x2A);
   spi_write(RF_DIR_OUT);
   spi_write(0xC2);
   spi_write(0xC2);
   spi_write(0xC2);
   spi_write(0xC2);
   RF_CS=1;

   // RX_ADDR_P1
   //Rhe own address is entered in RF_DIR. 
   //In this way the receiver knows the address of the transmitter.
   RF_CS=0;
   spi_write(0x0B);
   RF_PAY_DIR=spi_read(0);
   spi_read(0);
   spi_read(0);
   spi_read(0);
   spi_read(0);
   RF_CS=1;

   // W_TX_PAYLOAD
   //The data is sent to the transducer
   RF_CS=0;
   spi_write(0xA0);

   DATA_N_SND++;

   for(i=0;i<8;i++)
      RF_PAY_OUT[i]=RF_DATA_OUT[i];

   spi_write(DATA_N_SND);
   spi_write(RF_PAY_DIR);

   for (i=0;i<8;i++)
      spi_write(RF_PAY_OUT[i]);

   RF_CS=1;

   // CONFIG
   // Go to transmission mode.
   RF_CS=0;
   spi_write(0x20);
   spi_write(0x0E);
   RF_CS=1;

   // Pulse of start of shipment.
   RF_CE=1;
   Delay10TCYx(1);
   Delay1TCY();
   Delay1TCY();
   RF_CS=0;

   NO_RF=0;

   while (RF_IRQ) {
      NO_RF++;

      // If it does not answer in 7ms, it has not been sent.
      if(NO_RF==500)
	  {
      	break;
       }
   }

   // STATUS

   // Reading of the status in the status register.
   RF_CS=0;
   estado=spi_read(0x27);
   spi_write(0x70);
   RF_CS=1;

   // EN_RXADDR
   //Enable Pipe0,Pipe1 and Pipe2.
   RF_CS=0;
   spi_write(0x22);
   spi_write(0x07);
   RF_CS=1;

   // TX_FLUSH   
   // Cleaning the output FIFO
   RF_CS=0;
   spi_write(0xE1);
   RF_CS=1;

   // CONFIG
   // Switch to reception mode
   RF_CS=0;
   spi_write(0x20);
   spi_write(0x0F);
   RF_CS=1;

   // END
   RF_CE=1;

   Delay10TCYx (15);

	INTCONbits.INT0F=0;
	
   // If it does not answer in 7ms, it has not been sent.
   if(NO_RF==500 ||NO_RF==0)
	{	         
		INTCONbits.GIE = INTER_RF;
		return(2);
   }

  
   //state
   // Checking the STATUS register bits that indicate whether it has been received
   // ACK and if the retransmissions have been completed without any ACK.
   if ((estado&(1<<4))==0 && (estado&(1<<5))!=0)
	{
		INTCONbits.GIE = INTER_RF;
      return(0);
   }
   else
	{  	
		INTCONbits.GIE = INTER_RF;
      return(1);
   }
}
//*****************************************************



//*****************************************************
//*                 RF_RECEIVE()                      *
//*****************************************************
//*Description: This routine is responsible for       *
//*checking if there has been a reception and if so,  *
//*returns the received frame.                        *
//*****************************************************
//*Input variables:                                   *
//                                                    * 
//*Output variables:     -RF_DATA_IN[]                *
//*                      - RF_DIR_IN                  *
//*                                                   *
//*Output:         - 0: Correct and unique reception  *
//*                - 1: Correct and multiple reception*
//*                - 2: No reception has occurred     *
//*****************************************************

unsigned char RF_RECEIVE(unsigned char* RF_DIR_IN, unsigned char* RF_DATA_IN)
{
    unsigned char i;
   unsigned char mas;
   unsigned char estado;
   static unsigned char INT0E_FLAG;
   static unsigned char GIE_FLAG;

   if(INTCONbits.INT0E)
   {
      INT0E_FLAG=1;
	  INTCONbits.GIE=0;
   }
   else   
      INT0E_FLAG=0;   
   
   if(INTCONbits.GIE)   
      GIE_FLAG = 1;   
   else   
      GIE_FLAG = 0;   
			
   //FIFO_STATUS
   // Checking the status of the FIFO
   // reception to check for more data
   RF_CS=0;
   spi_write(0x17);
   mas=spi_read(0);
   RF_CS=1;

   if (RF_IRQ==1 &&  mas&1!=0)
   {  
		if(INT0E_FLAG == 1)		
			INTCONbits.INT0E = 1;	
			
		if(GIE_FLAG == 1)
			INTCONbits.GIE = 1;
	
		return (2);
   }

   //STATUS
   //Reading the STATUS record
   RF_CS=0;
   estado=spi_read(0x07);
   RF_CS=1;

   if(RF_IRQ==0 && estado&(1<<6)==0)
   {  
      if(INT0E_FLAG == 1)		
			INTCONbits.INT0E = 1;	
			
		if(GIE_FLAG == 1)
			INTCONbits.GIE = 1;
			
      return(2);
   }

   INTCONbits.INT0F=0;

   //R_RX_PAYLOAD
   //Reading of the received data.
   RF_CS=0;
   spi_write(0x61);
   DATA_N_RCV=spi_read(0);
   *RF_DIR_IN=spi_read(0);
   for (i=0;i<8;i++)
   {
      RF_DATA_IN[i]=spi_read(0);      
   }
   RF_CS=1;

   //STATUS
   //Reading and resetting the STATUS register
   RF_CS=0;
   estado=spi_read(0x27);
   spi_write(0x70);
   RF_CS=1;

   //FIFO_STATUS
   // Checking the status of the FIFO
   // reception to check for more data
   RF_CS=0;
   spi_write(0x17);
   mas=spi_read(0);
   RF_CS=1;

   if (mas&1==0)
   {
		if(INT0E_FLAG == 1)		
			INTCONbits.INT0E = 1;	
			
		if(GIE_FLAG == 1)
			INTCONbits.GIE = 1;
			
      return(1);
   }
   if(INT0E_FLAG == 1)		
		INTCONbits.INT0E = 1;	
		
	if(GIE_FLAG == 1)
		INTCONbits.GIE = 1;
		
  return(0);
}

//*****************************************************



//*****************************************************
//*      void spi_write(unsigned char data)           *
//*****************************************************
//*Description: Take out data by spi		          *
//*****************************************************
// * Input variables: - data                          *
// * Output variables:                                *
//*****************************************************
void spi_write(unsigned char data)
{
	SSP1BUF=data;                                                               //Load send data
	while(SSP1STATbits.BF!=1)	{};                                         	//wait
}

//*****************************************************



//*****************************************************
//*     unsigned char spi_read(unsigned char data)    *
//*****************************************************
//*Description: Take data by spi and return           *
//*****************************************************
//* Input variables:                - data            *
//* Output variables:               -Reading          *
//*****************************************************

unsigned char spi_read(unsigned char data)
{
	SSP1BUF=data;                                                               //Load send data
	while(SSP1STATbits.BF!=1)	{};                                             //wait
	return(SSP1BUF);
}


