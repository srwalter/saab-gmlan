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
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
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

void main(void) {
    uint32_t addr = 0;
    char buf[256];
    int i=0;
    
    // 9600 baud for UART
    OpenUSART(USART_TX_INT_OFF &
            USART_RX_INT_OFF &
            USART_ASYNCH_MODE &
            USART_EIGHT_BIT &
            USART_CONT_RX &
            USART_BRGH_HIGH,
            25);
    
    putsUSART("Hello\r\n");
    __delay_ms(1000);
    putsUSART("Hello 2\r\n");
    
    // Turn on LED
    TRISA &= ~(1 << 1);
    LATA |= 1 << 1;
    
    // Set tristate for CAN pins
    TRISB &= ~(1 << 2);
    TRISB |= 1 << 3;
    
    CANCON |= (4 << 5);
    while ((CANSTAT >> 5) != 4) {
        Delay1TCY();
    }
    putsUSART("Config mode\r\n");
    
    // Receive all messages
    RXB0CON |= 3 << 5;
        
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

    // Put the transceiver in normal mode
    TRISA &= ~(1 << 4);
    TRISA &= ~(1 << 5);
    LATA |= 1 << 4;
    LATA |= 1 << 5;
    
    // Accept all messages
    RXM0SIDH = 0;
    RXM0SIDL = 0;
    RXM0EIDH = 0;
    RXM1EIDH = 0;
    RXM0EIDL = 0;
    RXM1EIDL = 0;
    
    // Go to listen-only mode
    CANCON = 3 << 5;
    while ((CANSTAT >> 5) != 3) {
        Delay1TCY();
    }
    putsUSART("Listening\r\n");
    
    while (1) {
        i=0;
        while(!RXB0FUL) {
            i++;
            if (i == 1000) {
                sprintf(buf, "%02x\r\n", PIR3);
                putsUSART(buf);
                sprintf(buf, "%02x\r\n", COMSTAT);
                putsUSART(buf);
                sprintf(buf, "%02x\r\n", RXERRCNT);
                putsUSART(buf);
                i=0;
            }
            __delay_ms(1);
        }
        LATA ^= 1 << 1;
        addr = RXB0SIDH << 8;
        addr |= RXB0SIDL;
        if (addr & 0x0008) {
            addr &= ~0x0008;
            addr <<= 16;
            addr |= RXB0EIDH << 8;
            addr |= RXB0EIDL;
        }
        sprintf(buf, "%08x\r\n", addr);
        putsUSART(buf);
        RXB0FUL = 0;
    }
    
    return;
}
