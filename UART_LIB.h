#ifndef UART_LIB_H
#define	UART_LIB_H

#include <stdint.h>
#include <xc.h>
#include <stdio.h>
#include "config.h"

// Configuración baudrate 9600 @ 64MHz
#define BAUDRATE      9600
#define HIGHSPEED
#define VALBRGH      (uint16_t)( (_XTAL_FREQ/ (4.0*BAUDRATE) )-1 )

// Prototipos
void UART1_Init(void);
void UART1_Write(char data);
void UART1_Text(char *str);
void UART1_Println(char *str);
void putch(char txData);
char UART1_Read(void);
char UART1_DATA_READY(void);

#endif