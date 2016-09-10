
#include "VelocityController.h"

#include "DirController.h"

const char ledPin = 13;
int ledState=LOW;

ToggleButton rightBtn, leftBtn;

VelocityController velCtrl;
DirController dirCtrl;

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(9600); 
  setupDirController(&dirCtrl, 8, 2, 12, 11);
  setupVelController(&velCtrl, 9, 3, 6, 5, 1000);
  //
  
  pinMode(ledPin, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  controlVelocity(&velCtrl);
  updateDir(&dirCtrl);
  delay(10);
}

