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


#ifdef	__cplusplus
}
#endif

#endif	/* DUININHO_H */

