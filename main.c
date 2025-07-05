/*******************************************************************************
 * PROYECTO: Control Bluetooth PIC18F57Q43
 * ARCHIVO: main.c
 * 
 * DESCRIPCIÓN:
 * Sistema de control por Bluetooth usando módulo HC-05/HC-06
 * - Control de LED en pin RA0 via comandos Bluetooth
 * - Comunicación bidireccional con smartphone/PC
 * - Comandos AT para configuración del módulo Bluetooth
 * 
 * HARDWARE:
 * - Microcontrolador: PIC18F57Q43 @ 64MHz
 * - LED: RA0 (ánodo) con resistencia 330? a tierra
 * - Bluetooth: Módulo HC-05/HC-06 conectado via UART1
 *   * VCC: 3.3V o 5V (según módulo)
 *   * GND: VSS
 *   * TXD: RC3 (RX del PIC)
 *   * RXD: RC2 (TX del PIC)
 *   * EN/KEY: +3.3V para modo AT (opcional)
 * 
 * COMANDOS BLUETOOTH DISPONIBLES:
 * - "LED_ON" o "1"    : Encender LED
 * - "LED_OFF" o "0"   : Apagar LED  
 * - "LED_TOGGLE" o "T": Cambiar estado LED
 * - "STATUS" o "?"    : Estado actual del sistema
 * - "HELP" o "H"      : Lista de comandos
 * - "RESET"           : Reiniciar sistema
 * 
 * AUTOR: Ingeniero Embebido
 * FECHA: [Fecha actual]
 *******************************************************************************/

#include <xc.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "config.h"
#include "UART_LIB.h"

/*******************************************************************************
 * DEFINICIONES Y CONSTANTES
 *******************************************************************************/

// Control del LED en RA0
#define LED_BLUETOOTH_ON()      LATAbits.LATA0 = 1
#define LED_BLUETOOTH_OFF()     LATAbits.LATA0 = 0
#define LED_BLUETOOTH_TOGGLE()  LATAbits.LATA0 ^= 1
#define LED_BLUETOOTH_STATE()   LATAbits.LATA0

// Buffer para comandos recibidos
#define CMD_BUFFER_SIZE 32
#define MAX_CMD_LENGTH 16

// Estados del sistema
typedef enum {
    SYSTEM_INIT,
    SYSTEM_READY,
    SYSTEM_PROCESSING_CMD,
    SYSTEM_ERROR
} system_state_t;

/*******************************************************************************
 * VARIABLES GLOBALES
 *******************************************************************************/

// Buffer para comandos Bluetooth
char cmd_buffer[CMD_BUFFER_SIZE];
uint8_t cmd_index = 0;
bool cmd_ready = false;

// Estado del sistema
system_state_t current_state = SYSTEM_INIT;
uint32_t system_uptime = 0;

// Estadísticas del sistema
uint16_t commands_received = 0;
uint16_t commands_executed = 0;
uint8_t led_toggle_count = 0;

/*******************************************************************************
 * PROTOTIPOS DE FUNCIONES
 *******************************************************************************/

void System_Init(void);
void LED_RA0_Init(void);
void Bluetooth_Init(void);
void Process_Bluetooth_Command(char *command);
void Send_Welcome_Message(void);
void Send_Status_Info(void);
void Send_Help_Menu(void);
bool Parse_Received_Data(void);
void Clear_Command_Buffer(void);
void System_Reset(void);

/*******************************************************************************
 * FUNCIÓN PRINCIPAL
 *******************************************************************************/
void main(void) {
    
    // Inicialización del sistema
    System_Init();
    
    // Mensaje de bienvenida via Bluetooth
    Send_Welcome_Message();
    
    // Bucle principal
    while(1) {
        
        // Procesar datos recibidos por Bluetooth
        if(Parse_Received_Data()) {
            if(cmd_ready) {
                current_state = SYSTEM_PROCESSING_CMD;
                Process_Bluetooth_Command(cmd_buffer);
                Clear_Command_Buffer();
                current_state = SYSTEM_READY;
            }
        }
        
        // Indicador de vida (parpadeo cada 2 segundos)
        static uint32_t heartbeat_counter = 0;
        if(++heartbeat_counter >= 200000) {  // Ajustar según necesidad
            LED_Toggle();  // LED interno RF3 como heartbeat
            heartbeat_counter = 0;
            system_uptime++;
        }
        
        // Aquí se pueden agregar otras tareas del sistema
        
    } // Fin del bucle principal
}

/*******************************************************************************
 * INICIALIZACIÓN DEL SISTEMA
 *******************************************************************************/
void System_Init(void) {
    
    // Configurar oscilador y pines básicos
    ClockInit();    // 64MHz HFINTOSC
    PinInit();      // I2C, UART, LED RF3
    
    // Configurar LED en RA0 para control Bluetooth
    LED_RA0_Init();
    
    // Inicializar UART para Bluetooth
    Bluetooth_Init();
    
    // Inicializar variables
    Clear_Command_Buffer();
    current_state = SYSTEM_READY;
    
    // Delay inicial para estabilización
    __delay_ms(500);
}

/*******************************************************************************
 * CONFIGURACIÓN DEL LED EN RA0
 *******************************************************************************/
void LED_RA0_Init(void) {
    
    // Configurar RA0 como salida digital
    ANSELAbits.ANSELA0 = 0;    // RA0 como digital (no analógico)
    TRISAbits.TRISA0 = 0;      // RA0 como salida
    WPUAbits.WPUA0 = 0;        // Sin pull-up en RA0
    
    // Estado inicial del LED (apagado)
    LED_BLUETOOTH_OFF();
    
    // Test del LED (3 parpadeos rápidos)
    for(uint8_t i = 0; i < 3; i++) {
        LED_BLUETOOTH_ON();
        __delay_ms(100);
        LED_BLUETOOTH_OFF();
        __delay_ms(100);
    }
}

/*******************************************************************************
 * INICIALIZACIÓN BLUETOOTH
 *******************************************************************************/
void Bluetooth_Init(void) {
    
    // Inicializar UART1 con la librería existente
    UART1_Init();  // 9600 baudios, 8N1, RC2(TX), RC3(RX)
    
    // Delay para que el módulo Bluetooth se inicialice
    __delay_ms(1000);
    
    // Opcional: Configurar módulo HC-05 con comandos AT
    // Solo si el pin EN/KEY está conectado a +3.3V
    /*
    UART1_Println("AT");                    // Test de comunicación
    __delay_ms(100);
    UART1_Println("AT+NAME=PIC18F57Q43");   // Nombre del dispositivo
    __delay_ms(100);
    UART1_Println("AT+PSWD=1234");          // PIN de conexión
    __delay_ms(100);
    UART1_Println("AT+UART=9600,0,0");      // Configurar baudrate
    __delay_ms(500);
    */
}

/*******************************************************************************
 * MENSAJE DE BIENVENIDA
 *******************************************************************************/
void Send_Welcome_Message(void) {
    
    UART1_Println("\r\n" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=");
    UART1_Println("      SISTEMA BLUETOOTH PIC18F57Q43");
    UART1_Println("      Control de LED via Bluetooth");
    UART1_Println("=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=" "=");
    UART1_Println("");
    UART1_Println("Sistema inicializado correctamente.");
    UART1_Println("LED de control conectado en pin RA0.");
    UART1_Println("Frequency: 64MHz | UART: 9600 bps");
    UART1_Println("");
    UART1_Println("Escriba 'HELP' para ver comandos disponibles.");
    UART1_Println("");
}

/*******************************************************************************
 * MENÚ DE AYUDA
 *******************************************************************************/
void Send_Help_Menu(void) {
    
    UART1_Println("\r\n--- COMANDOS DISPONIBLES ---");
    UART1_Println("LED_ON  o 1      : Encender LED RA0");
    UART1_Println("LED_OFF o 0      : Apagar LED RA0");
    UART1_Println("LED_TOGGLE o T   : Cambiar estado LED");
    UART1_Println("STATUS o ?       : Información del sistema");
    UART1_Println("HELP o H         : Mostrar esta ayuda");
    UART1_Println("RESET            : Reiniciar sistema");
    UART1_Println("--- FIN COMANDOS ---\r\n");
}

/*******************************************************************************
 * INFORMACIÓN DEL ESTADO DEL SISTEMA
 *******************************************************************************/
void Send_Status_Info(void) {
    
    char status_str[50];
    
    UART1_Println("\r\n--- ESTADO DEL SISTEMA ---");
    
    // Estado del LED
    if(LED_BLUETOOTH_STATE()) {
        UART1_Println("LED RA0: ENCENDIDO");
    } else {
        UART1_Println("LED RA0: APAGADO");
    }
    
    // Estadísticas
    sprintf(status_str, "Comandos recibidos: %u", commands_received);
    UART1_Println(status_str);
    
    sprintf(status_str, "Comandos ejecutados: %u", commands_executed);
    UART1_Println(status_str);
    
    sprintf(status_str, "LED toggles: %u", led_toggle_count);
    UART1_Println(status_str);
    
    sprintf(status_str, "Uptime: %lu ciclos", system_uptime);
    UART1_Println(status_str);
    
    // Estado del sistema
    switch(current_state) {
        case SYSTEM_READY:
            UART1_Println("Estado: LISTO");
            break;
        case SYSTEM_PROCESSING_CMD:
            UART1_Println("Estado: PROCESANDO");
            break;
        case SYSTEM_ERROR:
            UART1_Println("Estado: ERROR");
            break;
        default:
            UART1_Println("Estado: DESCONOCIDO");
            break;
    }
    
    UART1_Println("--- FIN ESTADO ---\r\n");
}

/*******************************************************************************
 * PROCESAR COMANDOS BLUETOOTH
 *******************************************************************************/
void Process_Bluetooth_Command(char *command) {
    
    commands_received++;
    
    // Convertir a mayúsculas para comparación
    for(uint8_t i = 0; command[i] != '\0'; i++) {
        if(command[i] >= 'a' && command[i] <= 'z') {
            command[i] = command[i] - 32;
        }
    }
    
    // Procesar comandos
    if(strcmp(command, "LED_ON") == 0 || strcmp(command, "1") == 0) {
        LED_BLUETOOTH_ON();
        UART1_Println("-> LED ENCENDIDO");
        commands_executed++;
        
    } else if(strcmp(command, "LED_OFF") == 0 || strcmp(command, "0") == 0) {
        LED_BLUETOOTH_OFF();
        UART1_Println("-> LED APAGADO");
        commands_executed++;
        
    } else if(strcmp(command, "LED_TOGGLE") == 0 || strcmp(command, "T") == 0) {
        LED_BLUETOOTH_TOGGLE();
        led_toggle_count++;
        if(LED_BLUETOOTH_STATE()) {
            UART1_Println("-> LED ENCENDIDO (toggle)");
        } else {
            UART1_Println("-> LED APAGADO (toggle)");
        }
        commands_executed++;
        
    } else if(strcmp(command, "STATUS") == 0 || strcmp(command, "?") == 0) {
        Send_Status_Info();
        commands_executed++;
        
    } else if(strcmp(command, "HELP") == 0 || strcmp(command, "H") == 0) {
        Send_Help_Menu();
        commands_executed++;
        
    } else if(strcmp(command, "RESET") == 0) {
        UART1_Println("-> REINICIANDO SISTEMA...");
        __delay_ms(100);
        System_Reset();
        
    } else {
        // Comando no reconocido
        UART1_Text("-> Comando desconocido: ");
        UART1_Println(command);
        UART1_Println("   Escriba 'HELP' para ver comandos disponibles.");
    }
}

/*******************************************************************************
 * ANALIZAR DATOS RECIBIDOS
 *******************************************************************************/
bool Parse_Received_Data(void) {
    
    // Verificar si hay datos disponibles
    if(!UART1_DATA_READY()) {
        return false;
    }
    
    // Leer caracter
    char received_char = UART1_Read();
    
    // Echo del caracter (opcional - comentar si no se desea)
    UART1_Write(received_char);
    
    // Procesar caracter
    if(received_char == '\r' || received_char == '\n') {
        // Fin de comando
        if(cmd_index > 0) {
            cmd_buffer[cmd_index] = '\0';  // Terminar string
            cmd_ready = true;
            return true;
        }
    } else if(received_char == '\b' || received_char == 127) {
        // Backspace
        if(cmd_index > 0) {
            cmd_index--;
        }
    } else if(cmd_index < (CMD_BUFFER_SIZE - 2)) {
        // Caracter válido
        cmd_buffer[cmd_index++] = received_char;
    }
    
    return false;
}

/*******************************************************************************
 * LIMPIAR BUFFER DE COMANDOS
 *******************************************************************************/
void Clear_Command_Buffer(void) {
    for(uint8_t i = 0; i < CMD_BUFFER_SIZE; i++) {
        cmd_buffer[i] = 0;
    }
    cmd_index = 0;
    cmd_ready = false;
}

/*******************************************************************************
 * REINICIAR SISTEMA (SOFTWARE RESET)
 *******************************************************************************/
void System_Reset(void) {
    // Método 1: Reset por Watchdog
    WDTCON0bits.SWDTEN = 1;  // Habilitar WDT
    while(1);                // Esperar reset
    
    // Método 2: Reset por instrucción (alternativo)
    // asm("RESET");
}

/*******************************************************************************
 * INFORMACIÓN DE COMPILACIÓN
 *******************************************************************************/

#pragma message "===== CONFIGURACIÓN BLUETOOTH ====="
#pragma message "LED Control: Pin RA0"
#pragma message "UART Bluetooth: RC2(TX), RC3(RX) @ 9600 bps"
#pragma message "Sistema: PIC18F57Q43 @ 64MHz"
#pragma message "====================================="

/*******************************************************************************
 * NOTAS DE IMPLEMENTACIÓN:
 * 
 * 1. CONEXIÓN MÓDULO BLUETOOTH HC-05/HC-06:
 *    - VCC: 3.3V (HC-05) o 5V (HC-06)
 *    - GND: VSS del PIC
 *    - TXD: RC3 (pin 14 - RX del PIC)
 *    - RXD: RC2 (pin 13 - TX del PIC)
 *    - EN/KEY: Opcional para comandos AT
 * 
 * 2. LED INDICADOR:
 *    - Ánodo: RA0 (pin 2)
 *    - Cátodo: Resistencia 330? ? VSS
 * 
 * 3. APLICACIÓN MÓVIL RECOMENDADA:
 *    - "Bluetooth Terminal" (Android)
 *    - "BlueTerm" (iOS)
 *    - Cualquier terminal Bluetooth serie
 * 
 * 4. CONFIGURACIÓN BLUETOOTH:
 *    - Nombre: PIC18F57Q43 (configurable)
 *    - PIN: 1234 (configurable)
 *    - Baudrate: 9600 bps
 * 
 * 5. OPTIMIZACIONES POSIBLES:
 *    - Agregar buffer circular para RX
 *    - Implementar timeout para comandos
 *    - Agregar más periféricos (ADC, PWM, etc.)
 *    - Sistema de logging de eventos
 *******************************************************************************/