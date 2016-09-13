/* 
 * File:   duininho.h
 * Author: noface
 *
 * Created on 12 de Setembro de 2016, 15:08
 */

#ifndef DUININHO_H
#define	DUININHO_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <xc.h>

void delay(int ms){
    for(int a = 0; a < ms; a++){
        __delay_us(990);
    }
}

void setupPwm(){
    //pinModes
    TRISC1 = 0; //RC1 is CCP2
    TRISC2 = 0; //RC2 is CCP1
    PORTCbits.RC1 = 0;
    PORTCbits.RC2 = 0;
    
    /*
    * PWM registers configuration
    * Fosc = 1000000 Hz
    * Fpwm = 61.04 Hz (Requested : 61.04 Hz)
    * Duty Cycle = 100 %
    * Resolution is 10 bits
    * Prescaler is 16
    * Ensure that your PWM pin is configured as digital output
    * see more details on http://www.micro-examples.com/
    * this source code is provided 'as is',
    * use it at your own risks
    */
    PR2 = 0b11111111 ;
    T2CON = 0b00000111 ;
    CCP1CON = 0b00111100 ;
    CCP2CON = 0b00111100 ;
}

void analogWrite(char pin, char value){
    if(pin == 1){
        CCPR1L = value;
    }else{
        CCPR2L = value;
    }
}

int digitalRead(int pin){
    switch(pin){
        case 10: return PORTBbits.RB0;
        case 11: return PORTBbits.RB1;
        case 12: return PORTBbits.RB2;
        case 13: return PORTBbits.RB3;
        case 14: return PORTBbits.RB4;
        case 15: return PORTBbits.RB5;
        default: return 0;
    }
}

void digitalWrite(int pin, char value){
    switch(pin){
        case 10: PORTBbits.RB0=value; break;
        case 11: PORTBbits.RB1=value; break;
        case 12: PORTBbits.RB2=value; break;
        case 13: PORTBbits.RB3=value; break;
        case 14: PORTBbits.RB4=value; break;
        case 15: PORTBbits.RB5=value; break;
    }
}

void setupAnalog(){
    //clear before configure
    ADCON0 = 0;
    ADCON1 = 0;
    
    //enable analog reads
    ADCON0bits.ADON = 1;
    //negative reference is VSS (GND)
    ADCON1bits.VCFG1 = 0;
    //positive reference is VCC
    ADCON1bits.VCFG0 = 0;
    
    //Select the ADC clock period (TAD) based on frequency
    //values selected from datasheet
    const long MHZ = 1000000;
    const long freq = _XTAL_FREQ;
    
    char clockBits = 0; //ADCS
    if((freq >= 1 * MHZ) && (freq < 4*MHZ) ){
        clockBits = 0b000;
    }
    else if((freq >= 4*MHZ) && (freq < 16*MHZ)){
        clockBits = 0b001;
    }
    else if((freq >= 16*MHZ) && (freq < 64*MHZ)){
        clockBits = 0b010;
    }
    else if((freq >= 64*MHZ)){
        clockBits = 0b110;
    }
    else{
        clockBits = 0b111;
    }
    
    char acquisitionTimesConfig = 0b001; //2 TADs for acquisition time
    
    //analog result is 'right justified' 
    //i.e.: less significant bit is in first bit of ADRESL
    char adfmBit = 0b10000000;
    
    //compound ADCON2 bits
    ADCON2 =  (adfmBit)
            | (acquisitionTimesConfig << 3)
            | (clockBits);
    
}

/** Pin 0 corresponds to AN0, Pin 1 to AN1, and so on.
    Datasheet of PIC18F2XK20/4XK20 specify a (maximum) number of 12 analog inputs.
 */
unsigned analogRead(char analogPin){
    //set channel from analogPin
    ADCON0bits.CHS0 = analogPin & 0b0001;
    ADCON0bits.CHS1 = analogPin & 0b0010;
    ADCON0bits.CHS2 = analogPin & 0b0100;
    ADCON0bits.CHS3 = analogPin & 0b1000;
    
    //start analog read
    ADCON0bits.GO = 1;
    
    //wait the reading be complete
    while(ADCON0bits.GO){}
    
    unsigned result = ((unsigned)ADRESH) << 8; //high byte
    result |= ADRESL; //low byte
    
    return result;
}

int map(int value, int fromMin, int fromMax, int toMin, int toMax){
    int result = ((toMax - toMin) * value) / (fromMax - fromMin);
    return result + toMin;
}

#ifdef	__cplusplus
}
#endif

#endif	/* DUININHO_H */

