#include "Arduino.h"
#include "ToggleButton.h"
  
typedef struct{
  char pinRight, pinLeft;
  ToggleButton leftBtn, rightBtn;
} DirController;

void setupDirController(DirController * dirCtrl, 
  char leftBtnPin, char rightBtnPin,
  char leftOutPin, char rightOutPin )
{ 
  dirCtrl->pinLeft = leftOutPin;
  dirCtrl->pinRight = rightOutPin;
  setupButton(&dirCtrl->leftBtn, leftBtnPin);
  setupButton(&dirCtrl->rightBtn, rightBtnPin);
}

void updateDir(DirController * dirCtrl){
  char rightState = LOW;
  char leftState = LOW;
  if(isPressed(&dirCtrl->rightBtn)){
    rightState = HIGH;
  }
  else if(isPressed(&dirCtrl->leftBtn)){
    leftState = HIGH;
  }
  digitalWrite(dirCtrl->pinLeft, leftState);
  digitalWrite(dirCtrl->pinRight, rightState);
}
