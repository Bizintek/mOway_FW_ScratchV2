#ifndef __LIB_RF2GH4_H
    #define __LIB_RF2GH4_H
    
    #include "delays.h"                                                         //RF library

    void RF_CONFIG_SPI(void);
    void RF_INT_EN(void);
    unsigned char RF_CONFIG(unsigned char canal, unsigned char dir);
    unsigned char RF_ON(void);
    unsigned char RF_OFF(void);
    unsigned char RF_SEND(unsigned char RF_DIR_OUT, unsigned char RF_DATA_OUT[]);
    unsigned char RF_RECEIVE(unsigned char* RF_DIR_IN, unsigned char* RF_DATA_IN);
    void spi_write(unsigned char data);
    unsigned char spi_read(unsigned char data);

    //**************************************************
    //*                   DEFINITIONS                  *
    //**************************************************

    // PORTB
    #define   RF_IRQ        PORTBbits.RB0
    // PORTC
    #define   SCK           PORTCbits.RC3
    #define   SDI           PORTCbits.RC4
    #define   SDO           PORTCbits.RC5
    // PORTF
    #define   RF_CS         PORTFbits.RF2
    // PORTH
    #define   RF_CE         PORTHbits.RH4	

    // TRISB
    #define   RF_IRQ_TRIS   TRISBbits.TRISB0
    // TRISC
    #define   SCK_TRIS      TRISCbits.TRISC3
    #define   SDI_TRIS      TRISCbits.TRISC4
    #define   SDO_TRIS      TRISCbits.TRISC5
    // TRISF
    #define   RF_CS_TRIS    TRISFbits.TRISF2	
    // TRISH
    #define   RF_CE_TRIS    TRISHbits.TRISH4
    
    //************************************************//
#endif






