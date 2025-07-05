#include "UART_LIB.h"

void UART1_Init(void){
    // Limpiar registros
    U1RXB = 0x0;
    U1RXCHK = 0x0;
    U1TXB = 0x0;
    U1TXCHK = 0x0;
    U1P1L = 0x0;
    U1P1H = 0x0;
    U1P2L = 0x0;
    U1P2H = 0x0;
    U1P3L = 0x0;
    U1P3H = 0x0;
 
    // Configuración UART: 8N1, TX/RX enabled, alta velocidad
    U1CON0 = 0xB0;  // BRGS=1, TXEN=1, RXEN=1, MODE=0000
    U1CON1 = 0x80;  // ON=1
    U1CON2 = 0x08;  // C0EN=1
    
    // Baudrate 9600 @ 64MHz
    U1BRGH = (VALBRGH & 0xFF00) >> 8;
    U1BRGL = (VALBRGH & 0x00FF) >> 0;
    
    // Configuración FIFO e interrupciones
    U1FIFO = 0x2E;
    U1UIR = 0x00;
    U1ERRIR = 0x80;
    U1ERRIE = 0x00;
}

void UART1_Write(char data){
    while(U1ERRIRbits.TXMTIF == 0);
    U1TXB = data;
}

void UART1_Text(char *str){
    while(*str){
        UART1_Write(*str);
        str++;     
    }
}

void UART1_Println(char *str){
    UART1_Text(str);
    UART1_Write('\r');
    UART1_Write('\n');
}

void putch(char txData){
    UART1_Write(txData);
}

char UART1_Read(void){
    while(!PIR4bits.U1RXIF);
    return U1RXB;
}

char UART1_DATA_READY(void){
    return PIR4bits.U1RXIF;
}