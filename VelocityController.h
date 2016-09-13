/* 
 * File:   VelocityController.h
 * Author: noface
 *
 * Created on 12 de Setembro de 2016, 14:46
 */

#ifndef VELOCITYCONTROLLER_H
#define	VELOCITYCONTROLLER_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "duininho.h"
    
typedef struct{
  int directPin, revertPin;
  int vel, maxVel;
} VelocityController;

void setVel(VelocityController * vCtrl, int vel);

void setupVelocityController(VelocityController * vCtrl, char directPin, char reversePin){
    vCtrl->directPin = directPin;
    vCtrl->revertPin = reversePin;
    vCtrl->maxVel = 128;
    vCtrl->vel = 0;
    
    setupPwm();
    //setVel(vCtrl, 0);
}

void setVel(VelocityController * vCtrl, int vel){
    if(vel > vCtrl->maxVel){
        vel = vCtrl->maxVel;
    }
    else if(vel < -vCtrl->maxVel){
        vel = -vCtrl->maxVel;
    }
    
    vCtrl->vel = vel;
        
    analogWrite(vCtrl->directPin, vel > 0 ? vel : 0);
    analogWrite(vCtrl->revertPin, vel < 0 ? -vel : 0);
}


void accell(VelocityController * vCtrl, int v){
    setVel(vCtrl, vCtrl->vel + v);
}


#ifdef	__cplusplus
}
#endif

#endif	/* VELOCITYCONTROLLER_H */

