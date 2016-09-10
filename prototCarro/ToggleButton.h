#ifndef TOGGLE_BUTTON_H
#define TOGGLE_BUTTON_H

#include "Arduino.h"

#ifndef DEBOUNCE_TIME
#define DEBOUNCE_TIME 50
#endif

typedef struct{
  char pin;
  int lastChangeTime;
  char state :2, lastReading :2;
} ToggleButton; 

boolean setupButton(ToggleButton * btn, char pin){
  btn->pin = pin;
  btn->lastChangeTime = 0;
  btn->state = 0;
  btn->lastReading = 0;
  
  pinMode(pin, INPUT_PULLUP);
}

boolean checkToggle(ToggleButton * btn){
  char reading = digitalRead(btn->pin);
  
  if(reading != btn->state){
    if(reading != btn->lastReading){
      btn->lastChangeTime = millis();
      btn->lastReading = reading;
    }
    else if((millis() - btn->lastChangeTime) > DEBOUNCE_TIME){
      btn->state = reading;
      return true;
    }
  }
 
  return false; 
}

boolean isPressed(ToggleButton * btn){
  checkToggle(btn);
  return btn->state == 0;
}


#endif
