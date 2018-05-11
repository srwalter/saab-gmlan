/*
 * File:   newmain.c
 * Author: Tyler
 *
 * Created on May 6, 2018, 8:32 PM
 */

// PIC18F2480 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = BOHW     // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 0         // Brown-out Reset Voltage bits (VBOR set to 4.6V)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config BBSIZ = 1024     // Boot Block Size Select bit (1K words (2K bytes) boot block)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)

#include <xc.h>
#include <stdint.h>
#include <usart.h>
#include <stdio.h>

#define _XTAL_FREQ 4000000

#define CAN_MODE_LISTEN (0x03)
#define CAN_MODE_CONFIG (0x04)

void switch_can_mode(char mode) {
    CANCON = (mode << _CANCON_REQOP0_POSN);
    // XXX: apply mask?
    while ((CANSTAT >> _CANSTAT_OPMODE0_POSN) != mode) {
        Delay1TCY();
    }
}

void acc_on(void) {
    LATAbits.LA1 = 1;
}

void acc_off(void) {
    LATAbits.LA1 = 0;
}

void reverse_on(void) {
}

void reverse_off(void) {
}

char buf[256];

void handle_message(uint32_t addr) {
    switch (addr) {
        case 0x80002045:
            if (RXB0D0 == 0x00) {
                acc_off();
            }
            if (RXB0D0 >= 0x05) {
                acc_on();
            }
            break;
            
        case 0x81228045:
            if (RXB0D0 == 0x90) {
                reverse_on();
            } else {
                reverse_off();
            }
            break;
    }
}

void high_priority interrupt main_irq (void) {
    uint32_t addr = 0;
    
    if (PIR3bits.RXB0IF) {
        if (RXB0CONbits.RXFUL) {
            addr = RXB0SIDH;
            addr <<= 8;
            addr |= RXB0SIDL;
            if (addr & 0x8) {
                addr &= ~0x0018;
                addr <<= 8;
                addr |= RXB0EIDH;
                addr <<= 8;
                addr |= RXB0EIDL;
            }
#if 0
            sprintf(buf, "%08lx\r\n", addr);
            putsUSART(buf);
#endif
            handle_message(addr);
            RXB0CONbits.RXFUL = 0;
        }

        // Clear the interrupt
        PIR3bits.RXB0IF = 0;
    }
}

void main(void) {
    int i=0;
    
    // 9600 baud for UART
    OpenUSART(USART_TX_INT_OFF &
            USART_RX_INT_OFF &
            USART_ASYNCH_MODE &
            USART_EIGHT_BIT &
            USART_CONT_RX &
            USART_BRGH_HIGH,
            25);
    
    //putsUSART("Hello\r\n");
    __delay_ms(1000);
    //putsUSART("Hello 2\r\n");
    
    // Turn ACC off
    acc_off();
    reverse_off();
    TRISAbits.RA1 = 0;
    
    // TRISB<3> (CANRX) must be enabled for CAN operation
    TRISBbits.RB3 = 1;
    
    // Ensure we're in CAN config mode
    switch_can_mode(CAN_MODE_CONFIG);
    //putsUSART("Config mode\r\n");
       
    // Want 33333kbaud 
    // Nominal Bit Rate = 1/33333 = 30uS/bit
    // T_Q = 30uS / 15 Q = 2uS
    // BRP = (Fosc * T_Q / 2) - 1 = (4 * 2 / 2) - 1 = 3
    BRGCON1 = 3; // SJW = 1 T_Q BRP = 3
    BRGCON2 = 0; // Propagation = 1 T_Q
    BRGCON2 |= 6 << 3; // Phase 1 = 7 T_Q
    BRGCON2 |= 1 << 6;
    BRGCON2 |= 1 << 7;
    BRGCON3 = 0x05; // Phase 2 = 6 T_Q
    
    // Accept all messages
    RXM0SIDH = 0;
    RXM0SIDL = 0;
    RXM0EIDH = 0;
    RXM1EIDH = 0;
    RXM0EIDL = 0;
    RXM1EIDL = 0;

    // Filter 0 will accept standard messages and Filter 1 will accept extended messages
    RXF0SIDLbits.EXIDEN = 0;
    RXF1SIDLbits.EXIDEN = 1;
    
    // Go to listen-only mode
    switch_can_mode(CAN_MODE_LISTEN);
    //putsUSART("Listening\r\n");

    // Put the transceiver in normal mode
    LATAbits.LA4 = 1;
    LATAbits.LA5 = 1;
    TRISAbits.RA4 = 0;
    TRISAbits.RA5 = 0;

    // Make CAN-receive HIPRI
    IPR3bits.RXB0IP = 1;
    // Enable CAN-receive interrupts
    PIE3bits.RXB0IE = 1;
    // Enable interrupt priorities
    RCONbits.IPEN = 1;
    // Enable peripheral interrupts
    INTCONbits.PEIE = 1;
    // Enable global interrupts
    INTCONbits.GIEH = 1;
    
    // Keep peripherals running during sleep
    OSCCONbits.IDLEN = 1;
    while (1) {
        SLEEP();
    }
    
    return;
}
