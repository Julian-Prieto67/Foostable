#include <Arduino.h>
#include <digitalWriteFast.h>

long debugtimer = millis();

#define CW 0
#define CCW 1
#define DEBUGLED 7
#define LSPIN 3
const int epin = 2;
const int epintoggle = 5;
const int TABLEOFFSET = 25;

const double DIAMETER = 23;
const double CIRCUMFERENCE = DIAMETER * PI;

bool LEDFLASHER = false;
int tick = millis();
int clock = 0;

bool Calibrated = false; 

// Create a 2D array of bounds for each stepper
int gbounds[2];
int dbounds[2];
int mbounds[2];
int abounds[2];

//Decoding messages
char ENDCHAR = '#';
int counter = 0;
char Delimiter = '|';
String message = "0|0|0|0|";
String HOME = "HOM";
String SPEED = "SPD";
int LScounter = 0;



//all stepper stuff
int debugpos = 0;
int microstepping = 800;
const int MAXFREQ = 100000;                         //max frequency of the stepper
int StepperMotions[4] = { 0, 0, 0, 0 };             //array to hold the stepper motions
int StepDis[4] = { 0, 0, 0, 0 };                    //array to hold the actual stepper positions after bounding them or correcting
int StepperSpeeds[4] = { 2500, 2500, 2500, 2500 };  //array to hold stepper speeds
int initialStepperSpeeds[4] = { 500, 500, 500, 500 };  //array to hold stepper speeds
//Controls stuff

//////////// STEPPER 1 //////////////
bool Stepper1Running = false;     //flag to move the stepper
bool Stepper1State = false;       //Step pin state
bool Stepper1Dir = 0;             // flag to set the direction of the stepper
long Stepper1Count = 6000;           //position "count"
int stepper1[3] = { 0, 50, 52 };  //pos, step, dir
// int stepper1[3] = {0, 38, 40}; //pos, step, dir
long halfPeriod1 = 0;
long timer1 = 0;
//////////// STEPPER 2 //////////////
bool Stepper2Running = false;  //flag to move the stepper
bool Stepper2State = false;    //Step pin state
bool Stepper2Dir = 0;          // flag to set the direction of the stepper
long Stepper2Count = 9000;        //position "count"
int stepper2[3] = { 0, 46, 48 };
// int stepper2[3] = {0, 47, 49}; //pos, step, dir
long halfPeriod2 = 0;
long timer2 = 0;
//////////// STEPPER 3 //////////////
bool Stepper3Running = false;  //flag to move the stepper
bool Stepper3State = false;    //Step pin state
bool Stepper3Dir = 0;          // flag to set the direction of the stepper
long Stepper3Count = 6000;        //position "count"
int stepper3[3] = { 0, 42, 44 };
// int stepper3[3] = {0, 47, 49}; //pos, step, dir
long halfPeriod3 = 0;
long timer3 = 0;
//////////// STEPPER 4 //////////////
bool Stepper4Running = false;  //flag to move the stepper
bool Stepper4State = false;    //Step pin state
bool Stepper4Dir = 0;          // flag to set the direction of the stepper
long Stepper4Count = 60000;        //position "count"
int stepper4[3] = { 0, 38, 40 };
// int stepper4[3] = {0, 47, 49}; //pos, step, dir
long halfPeriod4 = 0;
long timer4 = 0;

float Kp = 2;

/////////debounce
long lastDebounceTime = 0;
int debounceDelay = 750;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// start functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////ESTOP///////////////////////////////////////////
void BoundTable(){
  gbounds[0] = TABLEOFFSET;
  gbounds[1] = 2.55 * microstepping - TABLEOFFSET;
  dbounds[0] = TABLEOFFSET;
  dbounds[1] = 5 * microstepping -TABLEOFFSET;
  mbounds[0] = TABLEOFFSET;
  mbounds[1] = 1.53125 * microstepping -TABLEOFFSET;
  abounds[0] = TABLEOFFSET;
  abounds[1] = 3.1375 * microstepping - TABLEOFFSET;
}
void ESTOP() {
  Serial.println("ESTOP");
  cli();  //disable other interrupts
  digitalWrite(DEBUGLED, HIGH);
  while (1)
    ;
}
void eStopSetup() {
  // ASSUMES THAT epin is hardware interrupt (ext int)
  pinMode(epin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(epin), ESTOP,  FALLING);
}
void debugMethod(int timepassed) {
  // for timepassed intervals, do something
  if (millis() - debugtimer > timepassed) {
    debugtimer = millis();
    ////insert code here
    debugpos = 100;
  } else if (millis() - debugtimer > timepassed / 2) {
    debugpos = 1000;
  }
}

//////////////////////////////LS Functions//////////////////////////////////////
flashLED(){
  //flashes the LED for debugging purposes
  if(LEDFLASHER){
    digitalWrite(DEBUGLED, HIGH);
  if (millis() - debugtimer > 1000){
    digitalWrite(DEBUGLED, LOW);
    LEDFLASHER = false;
  }
  }
}
void LSISR(){
  //if the limit switch is hit, set the stepper position to 0
  if (millis() - lastDebounceTime > debounceDelay){
    lastDebounceTime = millis();
    // LEDFLASHER = true;
    // debugtimer = millis();
    // Serial.println("Limit Switch Hit");
    if (LScounter ==0){
      Stepper1Count = 0;
    }
    if (LScounter ==1){
      Stepper2Count = 0;
    }
    if (LScounter ==2){
      Stepper3Count = 0;
    }
    if (LScounter ==3){
      Stepper4Count = 0;
    }
    LScounter++;
  delay(100);
  }
}
initializeSwitches(){
  //initialize the limit switches
  pinMode(LSPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LSPIN), LSISR, FALLING);
  
  //sei is called later
}
// Info for stepper1 in pos, step, dir

//////////////////////////////Stepper 1//////////////////////////////////////

void step1() {
  //non-blocking method of stepping the stepper.
  //toggles the step pin for stepper 1 if the appropriate time has passed
  // max frequency is something like 100 Khz before loop time is too slow
  int timerdif = micros() - timer1;
  if (Stepper1Running) {
    if (timerdif > halfPeriod1) {
      digitalWriteFast(stepper1[1], Stepper1State ? HIGH : LOW);
      Stepper1State = !Stepper1State;
      timer1 = micros();
      Stepper1Count += Stepper1Dir ? -1 : 1;
    }
  } else
    digitalWriteFast(stepper1[1], LOW);
}
void setSpeed1(float frequency) {
  // set the delay between peaks of the step signal
  // max frequency is something like 100 Khz before loop time is too slow
  // frequency must be positive
  frequency = abs(frequency);
  // Calculate the period of the signal in usecs
  unsigned long period = max(1E6 / frequency, 1E6 / MAXFREQ);
  // Calculate the half period in usecs to calculate time between toggles
  halfPeriod1 = period / 2;
}
void moveStepper1(int step) {
  //drive the value of Stepper1Count to step
  //USES step fxn to move the stepper

  int posDif = step - (int)(Stepper1Count * 0.5);
  // move the stepper to step
  if (posDif > 0) {
    if (posDif >200){
      setSpeed1(StepperSpeeds[0]+(int)Kp*posDif);
    }
    else{
      setSpeed1(StepperSpeeds[0]);
    }
    // Set the direction pin
    digitalWriteFast(stepper1[2], CW);
    Stepper1Dir = CW;
  } else {
    // Set the direction pin
    digitalWriteFast(stepper1[2], CCW);
    Stepper1Dir = CCW;
  }

  if ((posDif) != 0) {
    Stepper1Running = true;
    step1();
  } else
    Stepper1Running = false;
}
//////////////////////////////Stepper 2//////////////////////////////////////
void step2() {
  //non-blocking method of stepping the stepper.
  //toggles the step pin for stepper 1 if the appropriate time has passed
  // max frequency is something like 100 Khz before loop time is too slow
  int timerdif = micros() - timer2;
  if (Stepper2Running) {
    if (timerdif > halfPeriod2) {
      digitalWriteFast(stepper2[1], Stepper2State ? HIGH : LOW);
      Stepper2State = !Stepper2State;
      timer2 = micros();
      Stepper2Count += Stepper2Dir ? -1 : 1;
    }
  } else
    digitalWriteFast(stepper2[1], LOW);
}
void setSpeed2(float frequency) {
  // set the delay between peaks of the step signal
  // max frequency is something like 100 Khz before loop time is too slow
  // frequency must be positive
  frequency = abs(frequency);
  // Calculate the period of the signal in usecs
  unsigned long period = max(1E6 / frequency, 1E6 / MAXFREQ);
  // Calculate the half period in usecs to calculate time between toggles
  halfPeriod2 = period / 2;
}
void moveStepper2(int step) {
  //drive the value of Stepper1Count to step
  //USES step fxn to move the stepper

  int posDif = step - (int)(Stepper2Count * 0.5);
  // move the stepper to step
  if (posDif > 0) {
    if (posDif >200){
      setSpeed2(StepperSpeeds[1]+(int)Kp*posDif);
    }
    else{
      setSpeed1(StepperSpeeds[1]);
    }
    // Set the direction pin
    digitalWriteFast(stepper2[2], CW);
    Stepper2Dir = CW;
  } else {
    // Set the direction pin
    digitalWriteFast(stepper2[2], CCW);
    Stepper2Dir = CCW;
  }
  // Move the stepper
  // Serial.println(posDif);
  if ((posDif) != 0) {
    // String sendit = "step" + ENDCHAR;
    // Serial.println(sendit);
    Stepper2Running = true;
    step2();
  } else
    Stepper2Running = false;
}

//////////////////////////////Stepper 3//////////////////////////////////////

void step3() {
  //non-blocking method of stepping the stepper.
  //toggles the step pin for stepper 1 if the appropriate time has passed
  // max frequency is something like 100 Khz before loop time is too slow
  int timerdif = micros() - timer3;
  if (Stepper3Running) {
    if (timerdif > halfPeriod3) {
      digitalWriteFast(stepper3[1], Stepper3State ? HIGH : LOW);
      Stepper3State = !Stepper3State;
      timer3 = micros();
      Stepper3Count += Stepper3Dir ? -1 : 1;
    }
  } else
    digitalWriteFast(stepper3[1], LOW);
}
void setSpeed3(float frequency) {
  // set the delay between peaks of the step signal
  // max frequency is something like 100 Khz before loop time is too slow
  // frequency must be positive
  frequency = abs(frequency);
  // Calculate the period of the signal in usecs
  unsigned long period = max(1E6 / frequency, 1E6 / MAXFREQ);
  // Calculate the half period in usecs to calculate time between toggles
  halfPeriod3 = period / 2;
}
void moveStepper3(int step) {
  //drive the value of Stepper1Count to step
  //USES step fxn to move the stepper

  int posDif = step - (int)(Stepper3Count * 0.5);
  // move the stepper to step
  if (posDif > 0) {
    if (posDif >200){
      setSpeed3(StepperSpeeds[2]+(int)Kp*posDif);
    }
    else{
      setSpeed3(StepperSpeeds[2]);
    }
    // Set the direction pin
    digitalWriteFast(stepper3[2], CW);
    Stepper3Dir = CW;
  } else {
    // Set the direction pin
    digitalWriteFast(stepper3[2], CCW);
    Stepper3Dir = CCW;
  }
  // Move the stepper
  // Serial.println(posDif);
  if ((posDif) != 0) {
    // String sendit = "step" + ENDCHAR;
    // Serial.println(sendit);
    Stepper3Running = true;
    step3();
  } else
    Stepper3Running = false;
}
//////////////////////////////Stepper 4//////////////////////////////////////
void step4() {
  //non-blocking method of stepping the stepper.
  //toggles the step pin for stepper 1 if the appropriate time has passed
  // max frequency is something like 100 Khz before loop time is too slow
  int timerdif = micros() - timer4;
  if (Stepper4Running) {
    if (timerdif > halfPeriod4) {
      digitalWriteFast(stepper4[1], Stepper4State ? HIGH : LOW);
      Stepper4State = !Stepper4State;
      timer4 = micros();
      Stepper4Count += Stepper4Dir ? -1 : 1;
    }
  } else
    digitalWriteFast(stepper4[1], LOW);
}
void setSpeed4(float frequency) {
  // set the delay between peaks of the step signal
  // max frequency is something like 100 Khz before loop time is too slow
  // frequency must be positive
  frequency = abs(frequency);
  // Calculate the period of the signal in usecs
  unsigned long period = max(1E6 / frequency, 1E6 / MAXFREQ);
  // Calculate the half period in usecs to calculate time between toggles
  halfPeriod4 = period / 2;
}
void moveStepper4(int step) {
  //drive the value of Stepper4Count to step
  //USES step fxn to move the stepper

  int posDif = step - (int)(Stepper4Count * 0.5);
  // move the stepper to step
  if (posDif > 0) {
    if (posDif >200){
      setSpeed1(StepperSpeeds[3]+(int)Kp*posDif);
    }
    else{
      setSpeed1(StepperSpeeds[3]);
    }
    // Set the direction pin
    digitalWriteFast(stepper4[2], CW);
    Stepper4Dir = CW;
  } else {
    // Set the direction pin
    digitalWriteFast(stepper4[2], CCW);
    Stepper4Dir = CCW;
  }

  if ((posDif) != 0) {
    Stepper4Running = true;
    step4();
  } else
    Stepper4Running = false;
}

/////////////////////////////Initialize motors//////////////////////////////////
void zeroSteppers(){
  bool moveOn = false;
  //Calibrates stepper1
  
  setSpeed1(initialStepperSpeeds[0]);
  setSpeed2(initialStepperSpeeds[1]);
  setSpeed3(initialStepperSpeeds[2]);
  setSpeed4(initialStepperSpeeds[3]);
  while(Stepper1Count != 0){
    moveStepper1(0);
    delay(1);
  }
  setSpeed1(StepperSpeeds[0]);
  //make sure stepper 1 is off the LS
  while(Stepper1Count != 1550){
    moveStepper1(1550);
  }
  //calibrate stepper2
  delay(1000);
  while(Stepper2Count != 0){
    // moveStepper1(10);
    moveStepper2(0);
    delay(1);
    // delay(10);
  }
  setSpeed2(StepperSpeeds[1]);
  //make sure stepper 2 is off
  while(Stepper2Count != 3000){
    moveStepper2(3000);
  }
  delay(1000);
  //calibrate stepper3
  while(Stepper3Count != 0){
    // moveStepper1(10);
    // moveStepper2(10);
    moveStepper3(0);
    delay(1);
    // delay(10);
  }
  //step off for stepper 3
  setSpeed3(StepperSpeeds[2]);
  while(Stepper3Count != 800){
    moveStepper3(800);
  }
  delay(1000);
  while(Stepper4Count != 0){
    // moveStepper1(10);
    // moveStepper2(10);
    // moveStepper3(10);
    moveStepper4(0);
    delay(1);
    // delay(10);
  }
  //step off stepper 4
  setSpeed4(StepperSpeeds[3]);
  while(Stepper4Count != 200){
    moveStepper4(200);
  }
  setSpeed1(StepperSpeeds[0]);
  setSpeed2(StepperSpeeds[1]);
  setSpeed3(StepperSpeeds[2]);
  setSpeed4(StepperSpeeds[3]);
  Serial.println("RDY" +ENDCHAR);
}

/////////////////////////////Communications//////////////////////////////////

void seperateMessage(String message, char delimiter) {
  int start = 0;
  int end = message.indexOf(delimiter);  // Find the first occurrence of the delimiter
  counter = 0;
  String cmd = message.substring(0, 3);
  if (cmd.equalsIgnoreCase(HOME)) {
    zeroSteppers();
    Calibrated = true;

  } else if (cmd.equalsIgnoreCase(SPEED)) {
    while (end != -1) {
      long part = atoi(message.substring(start + 3, end).c_str());  // Extract the substring
      StepperSpeeds[counter] = part;
      start = end + 1;                          // Move the start position past the delimiter
      end = message.indexOf(delimiter, start);  // Find the next occurrence of the delimiter
      counter += 1;
    }
    setSpeed1(StepperSpeeds[0]);
    setSpeed2(StepperSpeeds[1]);
    setSpeed3(StepperSpeeds[2]);
    setSpeed4(StepperSpeeds[3]);
  } else {
    while (end != -1) {
      long part = atoi(message.substring(start, end).c_str());  // Extract the substring
      StepperMotions[counter] = part;
      start = end + 1;                          // Move the start position past the delimiter
      end = message.indexOf(delimiter, start);  // Find the next occurrence of the delimiter
      counter += 1;
    }
    // StepDis[0] = StepperMotions[0] + stepCorrection;
    StepDis[0] = StepperMotions[0];
    StepDis[1] = StepperMotions[1];
    StepDis[2] = StepperMotions[2];
    StepDis[3] = StepperMotions[3];
    // bound it to the table
    StepDis[0] = max(StepDis[0], gbounds[0]);
    StepDis[0] = min(StepDis[0], gbounds[1]);

    StepDis[1] = max(StepDis[1], dbounds[0]);
    StepDis[1] = min(StepDis[1], dbounds[1]);

    StepDis[2] = max(StepDis[2], mbounds[0]);
    StepDis[2] = min(StepDis[2], mbounds[1]);

    StepDis[3] = max(StepDis[3], abounds[0]);
    StepDis[3] = min(StepDis[3], abounds[1]);
  }
}



