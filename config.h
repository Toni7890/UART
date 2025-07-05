#ifndef CONFIG_H
#define	CONFIG_H

#define _XTAL_FREQ 64000000UL

// Control LED RF3 (interno)
#define LED_ON()        LATFbits.LATF3 = 0
#define LED_OFF()       LATFbits.LATF3 = 1
#define LED_Toggle()    LATFbits.LATF3 ^= 1

// Prototipos
void ClockInit(void);
void PinInit(void);

#endif