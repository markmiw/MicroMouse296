/*
 * File:   Micromouse.c
 * Author: kellyzheng
 *
 * Created on February 5, 2015, 11:09 AM
 */


#include <stdio.h>
#include <stdlib.h>
#include <pic18f4620.h>
#include <time.h>
#include <math.h>
//#include "timer.h"
//#include "motor.h"
#include <xc.h>
#include "ADC.h"
#define _XTAL_FREQ 32000000


#pragma config OSC = INTIO67    // Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = OFF      // MCLR Pin Enable bit (RE3 input pin enabled; MCLR disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

//void high_priority interrupt high_isr(void);
//void low_priority interrupt low_isr(void);


int delay_time = 0;
int adc_value = 0;

/*****************/
/*     Delay     */

/*****************/
void delay(unsigned int ms) {
    int x = 0;
    for (x = 0; x < ms; x++)
        __delay_ms(1);
}

int distance_ml, distance_mr, distance_fr, distance_fl; // distance left front, right front, middle right, middle left in cm
int adc_mr, adc_ml, adc_fr, adc_fl; // ADC value of left sensor looking front, right sensor looking front etc...
int start = 0;
int hasleftwall, hasrightwall, hasfrontwall;
int error;
int currentl = 0;
int currentr = 0;

/*****************/
/*   Interrupt   */

/*****************/
void interrupt isr() {



    /***********************/
    /* Timer 1: Left Motor */
    /***********************/

    //if timer1 flag = 1 and start indication = 1 then turn motor
    if (PIR1bits.TMR1IF == 1) {
        PIR1bits.TMR1IF == 0; // Turn flag off

        TMR1H = 0x63;
        TMR1L = 0xC0;

        if (currentl == 0) {
            LATEbits.LATE0 = 1;
            LATEbits.LATE1 = 0;
            LATEbits.LATE2 = 0;
            PORTAbits.RA5 = 0;
            currentl = 1;
        } else if (currentl == 1) {
            LATEbits.LATE0 = 0;
            LATEbits.LATE1 = 1;
            LATEbits.LATE2 = 0;
            PORTAbits.RA5 = 0;
            currentl = 2;
        } else if (currentl == 2) {
            LATEbits.LATE0 = 0;
            LATEbits.LATE1 = 0;
            LATEbits.LATE2 = 1;
            PORTAbits.RA5 = 0;
            currentl = 3;
        } else {
            LATEbits.LATE0 = 0;
            LATEbits.LATE1 = 0;
            LATEbits.LATE2 = 0;
            PORTAbits.RA5 = 1;
            currentl = 0;
        }

        T1CONbits.TMR1ON = 1; // Turn timer 1 on
    }

    /***********************/
    /* Timer 3: Right Motor */
    /***********************/
    if (PIR2bits.TMR3IF == 1) // If timer 3 flag is on and start is 1
    {
        PIR2bits.TMR3IF == 0; // Turn timer 3 flag off
        TMR3H = 0x63;
        TMR3L = 0xC0;


        if (currentr == 0) {
            PORTDbits.RD7 = 0;
            PORTDbits.RD4 = 0;
            PORTDbits.RD5 = 0;
            PORTDbits.RD6 = 1;
            currentr = 1;
        } else if (currentr == 1) {
            PORTDbits.RD7 = 0;
            PORTDbits.RD4 = 0;
            PORTDbits.RD5 = 1;
            PORTDbits.RD6 = 0;
            currentr = 2;
        } else if (currentr == 2) {
            PORTDbits.RD7 = 0;
            PORTDbits.RD4 = 1;
            PORTDbits.RD5 = 0;
            PORTDbits.RD6 = 0;
            currentr = 3;
        } else {
            PORTDbits.RD7 = 1;
            PORTDbits.RD4 = 0;
            PORTDbits.RD5 = 0;
            PORTDbits.RD6 = 0;
            currentr = 0;
        }
        T3CONbits.TMR3ON = 1; // Turn timer 3 on
    }
}

int main(int argc, char* argv) {
    //enable 8MHz internal clock
    OSCCON = 0b01111100;
    //enable internal PLL clock = 32MHz
    OSCTUNE = 0b01001111;



    ADCON0bits.ADON = 0; // Disable A/D module
    //ADCON0bits.CHS = 0b0001;
    //ADCON0bits.CHS = 0b0010;
    //ADCON0bits.CHS = 0b0011;
    ADCON1bits.VCFG1 = 0; // Use VSS for Vref- source
    ADCON1bits.VCFG0 = 0; // Use VDD for Vref+ source

    ADCON1bits.PCFG0 = 1; // Make AN0-AN3 Analog input rest digital
    ADCON1bits.PCFG1 = 0;
    ADCON1bits.PCFG2 = 1;
    ADCON1bits.PCFG3 = 1;

    ADCON2bits.ADFM = 1; // A/D result is right justified

    ADCON2bits.ACQT0 = 0; // Acquisition time
    ADCON2bits.ACQT1 = 1;
    ADCON2bits.ACQT2 = 0;

    ADCON2bits.ADCS0 = 0; // A/D conversion clock
    ADCON2bits.ADCS1 = 1;
    ADCON2bits.ADCS2 = 0;




    //Set Input/Output
    TRISA = 0b00001111; // input
    TRISC = 0b01100000;
    TRISB = 0b00000000;
    TRISE = 0b00000100;
    TRISD = 0b00010000;



    //Timer Interrupt Formula
    //Period = (4/FOSC)*Prescaler*(Resolution-Preload)
    // Period = (4/32000000)*Prescaler*(65536 - Preload)
    //Enable the Timer 0 interrupt


    RCONbits.IPEN = 0;


    // Timer0 Configuration Bits
    T0CONbits.TMR0ON = 0;
    T0CONbits.T08BIT = 1;
    T0CONbits.T0CS = 0;
    T0CONbits.T0SE = 0;
    T0CONbits.PSA = 0;
    T0CONbits.T0PS2 = 1;
    T0CONbits.T0PS1 = 1;
    T0CONbits.T0PS0 = 1;

    // Timer1 Configuration Bits

    /* PIR1 register */
    PIR1bits.PSPIF = 0;
    PIR1bits.ADIF = 0;
    PIR1bits.RCIF = 0;
    PIR1bits.TXIF = 0;
    PIR1bits.SSPIF = 0;
    PIR1bits.CCP1IF = 0;
    PIR1bits.TMR2IF = 0;
    PIR1bits.TMR1IF = 0;

    PIR2bits.OSCFIF = 0;
    PIR2bits.CMIF = 0;
    PIR2bits.EEIF = 0;
    PIR2bits.BCLIF = 0;
    PIR2bits.HLVDIF = 0;
    PIR2bits.TMR3IF = 0;
    PIR2bits.CCP2IF = 0;

    T1CONbits.TMR1ON = 0;
    T1CONbits.RD16 = 1; //read/write 16 bit values
    T1CONbits.T1RUN = 1; //Clock is derived from internal register
    T1CONbits.T1CKPS1 = 1; //1:2 prescaler
    T1CONbits.T1CKPS0 = 0;
    T1CONbits.T1OSCEN = 0; //oscillator shut off
    T1CONbits.TMR1CS = 0; //user internal clock (Fosc/4)
    T1CONbits.T1SYNC = 1;
    PIE1bits.TMR1IE = 1; //enable timer1 interrupt on overflow
    IPR1bits.TMR1IP = 0; //non high priority interrupt



    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;

    //Timer3 Configuration Bits
    T3CONbits.TMR3ON = 0;
    T3CONbits.RD16 = 1; //read/write 16 bit values
    T3CONbits.T3CKPS1 = 1; //1:2 prescaler
    T3CONbits.T3CKPS0 = 0;
    T3CONbits.T3SYNC = 1; //Do not sync external clock
    T3CONbits.TMR3CS = 0; //user internal clock (Fosc/4)
    PIE2bits.TMR3IE = 1; //enable timer1 interrupt on overflow
    IPR2bits.TMR3IP = 0; //non high priority interrupt


    // Interrupt Configuration Bits
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 0;
    INTCONbits.TMR0IE = 1;
    INTCONbits.INT0IE = 0;
    INTCONbits.RBIE = 0;
    INTCONbits.TMR0IF = 0; // Needs to be reset every flag


    PIE1bits.TMR1IE = 1;
    PIR1bits.TMR1IF = 0; // Needs to be reset every flag

    PIE2bits.TMR3IE = 1;
    PIR2bits.TMR3IF = 0; //clear interrupt flag



    ///////////////////////
    // Main Program Loop //
    ///////////////////////
    T1CON = 0x11;
    PIR1bits.TMR1IF == 0;
    TMR1H = 0x63;
    TMR1L = 0xC0;
    PIE1bits.TMR1IE == 1;
    INTCON = 0xC0;


    T3CON = 0x11;
    PIR2bits.TMR3IF == 0;
    TMR3H = 0x63;
    TMR3L = 0xC0;
    PIE2bits.TMR3IE == 1;
    INTCON3 = 0xC0;


    // Turn TMR0 on
    T0CONbits.TMR0ON = 1;

    //Turn TMR1 on
    T1CONbits.TMR1ON = 1;

    //Turn TMR3 on
    T3CONbits.TMR3ON = 1;

    //Start ADC
    ADCON0bits.ADON = 1; // Enable A/D module


    while (1) {

    }
}
