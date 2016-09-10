#include "Arduino.h"
#include "ToggleButton.h"

typedef struct{
  int vel, maxVel;
  int directPin, revertPin;
  ToggleButton accelBtn, r_accelBtn;
} VelocityController;

void setupVelController(VelocityController * ctrl, 
  int accelInPin, int r_accelInPin, 
  int directOutPin, int revertOutPin,
  int maxVel)
{
  ctrl->directPin = directOutPin;
  ctrl->revertPin = revertOutPin;
  
  pinMode(ctrl->directPin, OUTPUT);
  pinMode(ctrl->revertPin, OUTPUT);
  
  setupButton(&ctrl->accelBtn, accelInPin);
  setupButton(&ctrl->r_accelBtn, r_accelInPin);
  
  ctrl->vel=0;
  ctrl->maxVel = maxVel;
} 

void controlVelocity(VelocityController * ctrl){
  if(isPressed(&ctrl->accelBtn) && ctrl->vel < ctrl->maxVel){
    ++ctrl->vel; 
    digitalWrite(13, HIGH);
  }
  else if(isPressed(&ctrl->r_accelBtn) && ctrl->vel > -ctrl->maxVel){
    --ctrl->vel;
    digitalWrite(13, HIGH);
  }
  else{
    digitalWrite(13, LOW);
    
#ifdef IGNORE_FRICTION
    return;
#endif

    if(ctrl->vel == 0){
      return;
    }
    
    int friction = (int)(0.02f * ctrl->vel);
    ctrl->vel -= friction;
  }
  
  int mapped = map(abs(ctrl->vel), 0, ctrl->maxVel, 0, 255);
  
  analogWrite(ctrl->directPin, ctrl->vel > 0 ? mapped : LOW);
  analogWrite(ctrl->revertPin, ctrl->vel < 0 ? mapped : LOW);
}
