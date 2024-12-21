// The main file for the project as of 12-17-2024
#include <Arduino.h>
#include "Functions.h"


void setup()
{
  Serial.begin(115200);
  message.reserve(30);

  pinMode(DEBUGLED, OUTPUT); //this is the debugging LED 

  pinMode(epintoggle, OUTPUT);
  digitalWrite(epintoggle, HIGH);
  
  // Set the Stepper pins as outputs
  pinMode(stepper1[1], OUTPUT);
  pinMode(stepper1[2], OUTPUT);
  pinMode(stepper2[1], OUTPUT);
  pinMode(stepper2[2], OUTPUT);
  pinMode(stepper3[1], OUTPUT);
  pinMode(stepper3[2], OUTPUT);
  pinMode(stepper4[1], OUTPUT);
  pinMode(stepper4[2], OUTPUT);

  initializeSwitches();
  // debugpos = 1000;
  // eStopSetup();  
  sei();
  BoundTable();
  
  // LEDFLASHER = true;
  // debugtimer = millis();
  setSpeed1(StepperSpeeds[0]);
  setSpeed2(StepperSpeeds[1]);
  setSpeed3(StepperSpeeds[2]);
  setSpeed4(StepperSpeeds[3]);
}
void loop()
{

  clock = micros() - tick;
  tick = micros();
  // flashLED();

  if (Serial.available()) {
    // Read the incoming data as a string
    message = Serial.readStringUntil(ENDCHAR);
    seperateMessage(message, Delimiter);
    // Serial.println(clock);
    Serial.read();
  }
  // if (millis()- debugtimer > 1000){
  //   Serial.println(Stepper1Count);
  //   debugtimer = millis();
  // }
  // moveStepper1(debugpos);
  // debugMethod(5000);
  if(Calibrated){
  moveStepper1(StepDis[0]);
  moveStepper2(StepDis[1]);
  moveStepper3(StepDis[2]);
  moveStepper4(StepDis[3]);
}
}
