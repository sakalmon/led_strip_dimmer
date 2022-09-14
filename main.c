/*
 * File:   main.c
 * Author: Sakal
 *
 * Created on 3 November 2020, 9:28 PM
 */

#pragma config FOSC = INTIO67   // Oscillator Selection bits (Internal oscillator with port function on RA6 and RA7)
#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT0 = OFF       // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config WRT1 = OFF
#pragma config WRTC = OFF
#pragma config WRTB = OFF
#pragma config WRTD = OFF

#define _XTAL_FREQ 64000000
#define SSDI PORTDbits.RD0
#define RPI PORTBbits.RB4
#define WRITECOMMAND 0x3AA

#include <xc.h>
char pulses = 0;
char devices = 3;                       // Number of TLC5973s cascaded.
char devicesWritten = 0;
int b = 0;
int adchigh = 0, adclow = 0, adcval = 0;

int data[48] = {0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // First 2 bits are Tcycle, next 12 bits are the write command, remaining 36 bits are GSDATA  

void __interrupt() myint()
{
    T0CON = 0xC8;                       // Ensure timer 0 is set to 8-bit mode with no pre-scaler.
    
    if(INTCONbits.TMR0IF)
    {     
        SSDI = 1;
        SSDI = 0;        
        INTCONbits.TMR0IF = 0;
        pulses++;       
        
        if(pulses <= 3)
            TMR0 = 139;                 // Compensating for sync delay for the first two pulses
        
        else
            TMR0 = 135;       
        
        if(data[pulses - 1] == 1)
        {
            SSDI = 1;
            SSDI = 0;
            TMR0 = 164;
        }                
        if(pulses == 48)                // EOS
        {    
            devicesWritten++;
            if(devicesWritten == devices)
            {   
                devicesWritten = 0;
                T0CONbits.TMR0ON = 0;
                pulses = 0;
                __delay_us(400);        // GSLAT
                T0CONbits.TMR0ON = 1;
            }
            else
            {
                T0CONbits.TMR0ON = 0;
                T0CON = 0x41;           // Set timer 0 prescaler to 1:4 to allow for 40us EOS pulse 
                TMR0 = 95;              // Pre-load value for 40us overflow.
                pulses = 0;
                T0CONbits.TMR0ON = 1;
            }
        }                
    }    
}    

void brightness(int value)
{
    int i, j, k;
    
    j = 12;
    
    for(k = 0; k < 3; k++)
    {
        for(i = 11; i >= 0; i--)                
        {
            if(value & (1 << i))
                data[j] = 1;
            else
                data[j] = 0;

            j++;
        }   
    }
}
void main(void)
{
    OSCCON = 0xF4;                          // Idle enabled, 16 Mhz, internal oscillator, HFINTOSC stable, primary clock determined by CONFIG1H[FOSC<3:0>].
    OSCTUNEbits.PLLEN = 1;                  // Enable PLL for 64 Mhz clock.
    
    INTCON = 0xE0;   
    
//    TRISBbits.RB4 = 1;                      // Set pin as input for raspberry pi output
                       // Enable global/peripheral interrupts, Enable overflow interrupt, Enable port B change interrupt.
//    INTCON2bits.RBPU = 0;                   // Enable port B pullups
//    WPUBbits.WPUB4 = 1;                     // Enable weak pull ups on port B
    
    TRISA |= 1;                             // Disable RA0 output driver.
    ANSEL |= 1;                             // Disable RA0 digital input buffer.
    ADCON0 = 0x00;                          // Select channel 0 (AN0).
    ADCON1 = 0x00;                          // Use internal VDD and VSS and ADC reference.
    ADCON2 = 0xBE;                          // Right justified, 20TAD, FOSC/64.
    ADCON0bits.ADON = 1;                    // Enable ADC.   
    
    TRISD = 0x00;
    PORTD = 0x00;
    
    T0CON = 0x48;                           // Disable Timer0, 8-bit mode, Internal Instruction clock, No pre-scaler.
    
    
// -----------------------------------------------------------------------------   
// Setup serial port - For debugging 
//------------------------------------------------------------------------------    
//SPBRG = 103;                            // Set 9600 baud rate
//TXSTAbits.TXEN = 1;                     // Enable transmitter
//TXSTAbits.SYNC = 0;                     // Asynchronous mode
//BAUDCONbits.BRG16 = 0;                  // 8-bit baud rate
//RCSTAbits.SPEN = 1;                     // Enable serial port   
// -----------------------------------------------------------------------------
    
    __delay_ms(500);
    
    T0CONbits.TMR0ON = 1;                   // Enable Timer0 after a short delay.   
    
    while(1)
    {          
        if(RPI)
        {    
            ADCON0bits.GO = 1;                  // Start conversion    
            adchigh = ADRESH;                   // Take a copy of ADC result high register
            adclow = ADRESL;                    // Take a copy of ADC result low register
            adcval = adclow | (adchigh << 8);   // Combine the high and low registers
            
            if((adcval * 4) > 4095)
                brightness(4095);
            else
                brightness((adcval * 4);   // Convert ADC 10 bit values to 12 bit values for LED driver.
        }
        
        else
            brightness(0);
    }        
// -----------------------------------------------------------------------------        
// For debugging        
// -----------------------------------------------------------------------------
//        for(int c = 0; c < 48; c++)
//        {
//            if(data[c] & 1)
//                TXREG = '1';
//            else
//                TXREG = '0';
//            
//            __delay_ms(10);
//        }
//        TXREG = 10;
//        __delay_ms(10);
//        TXREG = 13;
//        __delay_ms(100);
// -----------------------------------------------------------------------------
    }
        
        
        
        
        
}    


    
    
    
    
    
    
    
    

