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


/*** 3, 6, 8
 * controle
 *  Interno         -   Saida/Pino
 *  1 (vermelho)    -   5 (VCC)
 *  2 (marrom)      -   1
 *  3 (laranja)     -   2
 *  4 (amarelo)     -   
 *  5 (preto)       -   4 (GND)
 *  6 (azul)        -   7
 *  7 (verde)       -   9
 *  8 (cinza)       -   
 * 
 * controle pinos:
 * LX   - 1
 * LY   - 
 * RX   - 2
 * RY   - 9
 * GND  - 4
 * VCC  - 5
 
 */

#define DIR_OUT1 15
#define DIR_OUT2 14

#define VEL_OUT1_PWM 2
#define VEL_OUT2_PWM 1


#define PIN_AnalogX 0
#define PIN_AnalogY 1

VelocityController ctrl;
int lastState = 0;
    
void setupDirection();
void updateDirection(int dx);
int readDeltaX();

void setupVelocity();
void updateVelocity(int dy);
int readDeltaY();

void setup();
void loop();
void setupInput();

void main(){
    setup();
    
    while(1){
        loop();
    }
}

void setup(){
    setupVelocity();
    setupDirection();
    setupInput();
}

void loop(){
    int dx = readDeltaX();
    delay(1);
    int dy = readDeltaY();
    
    updateVelocity(dy);
    updateDirection(dx);
    delay(20);
}

void setupInput(){
    //analog pins (AN0, AN1)
    TRISA0 = 1; 
    TRISA1 = 1;
    
    setupAnalog();
}

int debounceAnalogRead(int pin){
    int count = 0;
    unsigned lastRead = 0;
    do{
        unsigned input = analogRead(pin);
        if(lastRead == 0 || (abs(input - lastRead) < 70)){
            ++count;
        }
        else{
            count = 0;
        }
        lastRead = input;
    }while(count < 3);
        
    return lastRead;
}

int readDeltaX(){
    unsigned xAnalog = analogRead(PIN_AnalogX);
//    unsigned xAnalog = debounceAnalogRead(PIN_AnalogX);
    int slice = 260, xSlice = 50; //valores calibrados manualmente
    
    return xAnalog <= xSlice ? -1 : (xAnalog >= 1023 - slice ? 1 : 0);
}
int readDeltaY(){
    unsigned yAnalog = analogRead(PIN_AnalogY);
    
    int slice = 1024/3;
    
    return yAnalog <= slice ? -1 : (yAnalog >= 1023 - slice ? 1 : 0);
}

void setupDirection(){
    //pinos de output
    TRISB5 = 0;
    TRISB4 = 0;
}

void updateDirection(int dx){
    char v1 = dx > 0 ? 1 : 0;
    char v2 = dx < 0 ? 1 : 0;
    
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
  
    setupVelocityController(&ctrl, VEL_OUT1_PWM, VEL_OUT2_PWM);
}

void updateVelocity(int dy){
    RD0 = 0;
    RD1 = 0;
    RD2 = 0;
    int currentState;
    if(dy != 0){
        // Estamos acelerando
        if(dy > 0){
            currentState = 1;
        }else if(dy < 0){
            currentState = -1;
        }
        
        if(currentState != lastState){
            ctrl.vel = 0;
        }
        accell(&ctrl, dy*3);
        RD1= 1;
    }
    else{
        currentState = 0;
        RD2 = 1;

        //desacelera
        for(int i=0; i < 3; ++i){
            if(ctrl.vel > 0){
                accell(&ctrl, -1);
            }
            else if(ctrl.vel < 0){
                accell(&ctrl, 1);
            }
        }
    }
    
    lastState = currentState;
}

/*
 
void setupLedsBar(){
    //leds
    TRISD = 0; //saida
    PORTD = 0;
    
    //analog pins (AN0, AN1)
    TRISA0 = 1; 
    TRISA1 = 1;
    
    setupAnalog();
}
void ledsBar1(){
    unsigned analog = analogRead(0);
//        unsigned analog = 500;
    int ledsBits = map(analog, 0, 1023, 0, 8);

    char ledsOut = 0;
    for(int i=0; i < ledsBits; ++i){
        ledsOut = (ledsOut << 1) | 1;
    }

    PORTD = ledsOut;
}
void ledsBar2(){
    int dx = readDeltaX();
    delay(1);
    int dy = readDeltaY();
    
    int pinX = dx + 2, pinY = dy + 5;
    
    PORTD = (1 << pinX) | (1 << pinY);
}

void main__(){
    setupLedsBar();
    
    while(1){
        ledsBar2();
        delay(5);
    }
}
 */