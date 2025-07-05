#include <xc.h>
#include "config.h"

// Configuration bits - PIC18F57Q43
#pragma config FEXTOSC = OFF
#pragma config RSTOSC = HFINTOSC_64MHZ
#pragma config CLKOUTEN = OFF
#pragma config CSWEN = ON
#pragma config FCMEN = ON
#pragma config MCLRE = EXTMCLR
#pragma config PWRTS = PWRT_OFF
#pragma config MVECEN = ON
#pragma config LPBOREN = OFF
#pragma config BOREN = SBORDIS
#pragma config BORV = VBOR_1P9
#pragma config ZCD = OFF
#pragma config PPS1WAY = ON
#pragma config STVREN = ON
#pragma config LVP = ON
#pragma config XINST = OFF
#pragma config WDTCPS = WDTCPS_31
#pragma config WDTE = OFF
#pragma config WDTCWS = WDTCWS_7
#pragma config WDTCCS = SC
#pragma config BBSIZE = BBSIZE_512
#pragma config BBEN = OFF
#pragma config SAFEN = OFF
#pragma config DEBUG = ON
#pragma config WRTB = OFF
#pragma config WRTC = OFF
#pragma config WRTD = OFF
#pragma config WRTSAF = OFF
#pragma config WRTAPP = OFF
#pragma config CP = OFF

void ClockInit(){
    OSCFRQ = 0x08;              // 64MHz
    OSCCON1 = 0x60;             // HFINTOSC
    OSCEN = 0x40;               // Enable HFINTOSC
    while((OSCCON3 & 0x10) == 0); // Wait ORDY = 1
}

void PinInit(){
    // LED interno RF3
    ANSELF = 0x00;
    TRISFbits.TRISF3 = 0;
    LATFbits.LATF3 = 1;
    
    // UART pines RC2(TX), RC3(RX)
    ANSELCbits.ANSELC2 = 0;     // RC2 digital
    ANSELCbits.ANSELC3 = 0;     // RC3 digital
    TRISCbits.TRISC2 = 0;       // RC2 salida (TX)
    TRISCbits.TRISC3 = 1;       // RC3 entrada (RX)
    WPUCbits.WPUC3 = 1;         // Pull-up RC3
    
    // UART PPS
    RC2PPS = 0x20;              // RC2 -> UART1 TX
    U1RXPPS = 0x13;             // RC3 -> UART1 RX
}