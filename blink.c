/* 
 * File:   Interrupts.c
 * Author: Julio
 *
 * Created on 8. September 2016, 09:36
 */

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1H
#pragma config FOSC = INTIO7     // Oscillator Selection bits (External RC oscillator, port function on RA6)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 18        // Brown Out Reset Voltage bits (VBOR set to 1.8 V nominal)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT is controlled by SWDTEN bit of the WDTCON register)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config HFOFST = OFF     // HFINTOSC Fast Start-up (The system clock is held off until the HFINTOSC is stable.)
#pragma config MCLRE = OFF      // MCLR Pin Enable bit (RE3 input pin enabled; MCLR disabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection Block 0 (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection Block 1 (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection Block 2 (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection Block 3 (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

#include "pic18f45k20.h"

#define _XTAL_FREQ 1000000


#include "duininho.h"
#include "VelocityController.h"


/***
 
 * controle pinos:
 * 1 - X
 * 3 - Y
 * GND - 4
 * VCC - 5
 
 */

#define DIR_IN1 12
#define DIR_IN2 13
#define DIR_OUT1 14
#define DIR_OUT2 15

int digitalRead(int pin){
    switch(pin){
        case 12: return PORTBbits.RB2;
        case 13: return PORTBbits.RB3;
        case 14: return PORTBbits.RB4;
        case 15: return PORTBbits.RB5;
        default: return 0;
    }
}

int digitalWrite(int pin, char value){
    switch(pin){
        case 12: PORTBbits.RB2=value; break;
        case 13: PORTBbits.RB3=value; break;
        case 14: PORTBbits.RB4=value; break;
        case 15: PORTBbits.RB5=value; break;
    }
}

VelocityController ctrl;
    
void setupDirection();
void updateDirection();

void setupVelocity();
void updateVelocity();

void main(){
    TRISC = 0 ;                     // set PORTC as output
    PORTC = 0 ;                     // clear PORTC
    
    //leitura pinos
    RBPU = 0;   //habilita pull-up em pinos
    
    setupVelocity();
    setupDirection();
    
    while(1){
        updateVelocity();
        updateDirection();
        delay(20);
    }
}


void setupDirection(){
    //leds
    TRISD5 = 0; //saida
    TRISD6 = 0; //saida
    
    //configura pinos de input
    TRISB2 = 1;
    WPUB2 = 1;
    
    TRISB3 = 1;
    WPUB3 = 1;
    
    //pinos de output
    TRISB5 = 0;
    TRISB4 = 0;
}

void updateDirection(){
    char v1 = 0;
    char v2 = 0;
    
    if(!digitalRead(DIR_IN1)){
        v1 = 1;
    }
    else if(!digitalRead(DIR_IN2)){
        v2 = 1;
    }
    
    RD5=v1;
    RD6=v2;
    
    digitalWrite(DIR_OUT1,v1);
    digitalWrite(DIR_OUT2,v2);
}

void setupVelocity(){
    //leds
    TRISD0 = 0; //saida
    TRISD1 = 0; //saida
    TRISD2 = 0; //saida
    
    //configura pinos de input
    TRISB0 = 1;
    WPUB0 = 1;
    
    TRISB1 = 1;
    WPUB1 = 1;
        
    setupPwm();
    
    analogWrite(1, 0);
    analogWrite(2, 0);
  
    setupVelocityController(&ctrl);
    
    setVel(&ctrl, 0);
}

void updateVelocity(){
    RD0 = 0;
    RD1 = 0;
    RD2 = 0;

    if(!RB0){
        accell(&ctrl, 1);
        RD0 = 1;
    }
    else if(!RB1){
        accell(&ctrl, -1);
        RD1 = 1;
    }
    else{
        RD2 = 1;

        if(ctrl.vel > 0){
            accell(&ctrl, -1);
        }
        else if(ctrl.vel < 0){
            accell(&ctrl, 1);
        }
    }
}