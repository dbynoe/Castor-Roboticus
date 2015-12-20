// INCLUDES

#include "FastLED.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "RunningAverage.h"
#include <USBSabertooth.h>

// PINS 


#define batSensor A2 //battery current sensor
#define eStop A3 //physical estop switch to disable motors
//neopixel setup
#define LED_DATA_PIN 4 //pin for neopixel ring
#define NUM_LEDS 24 //how many leds
#define OLED_DC     9 //oled pins
#define OLED_CS     0
#define OLED_RESET  10
#define xIn A1 //joystick pins
#define yIn A0

//SETTINGS

#define updateRate 40 //update rate for the motors, display, etc
#define displayRate 4000 //how many ms between display changes
#define bMid 512 //mid point of battery sensor
#define bRatio 8.192 //arduino units to amps ratio 40 mv/a
//two drive modes, normal and voyageur
//voyageur is activated by releasing the estop with the stick in the lower left corner
//settings below configure the drive feel 
//normal mode:
#define dDecelRampN 3 //multiplier for how much faster to decel than accel for drive and reverse
#define tDecelRampN 20 //same as above for turning
#define dStepLowN  1 //minimum change to drive per cycle - stick response is proportional
#define dStepHighN 100 //maximum change to drive per cycle 
#define tStepLowN 1 //minimum turn change
#define tStepHighAN 20 //maximum - changes based on drive position
#define tStepHighBN 15 //same as above but for when drive is high
#define dMaxN  950 //maximum output for drive
#define tMaxN  650 //maximum output for turn
//voyageur mode:
#define dDecelRampV 1.8
#define tDecelRampV 25
#define dStepLowV 1
#define dStepHighV 350
#define tStepLowV 1
#define tStepHighAV 25
#define tStepHighBV 10
#define dMaxV  2048
#define tMaxV  2048

//GLOBAL VARIABLES
int dDecelRamp  = 1.5; 
int tDecelRamp = 20;
int dStepLow;
int dStepHigh;
int tStepLow;
int tStepHighA;
int tStepHighB;
int dMax;
int tMax;
int yCen = 499;
int xCen = 516;

int dOutput;  //drive output variable
int tOutput; //turn output
int dInput; //drive input
int tInput; //turn input
int dStep; //how much to change drive
int tStep; //how much to change turn


Adafruit_SSD1306 display(OLED_DC, OLED_RESET, OLED_CS);

boolean voyageur = false; //setting for advanced speed/driving mode
boolean eStopAct = false; //estop flag

RunningAverage batteryRA(5); //running averages for smoothing out jittery inputs and outputs
RunningAverage motor1RA(10);
RunningAverage motor2RA(10);
RunningAverage dInputRA(3);
RunningAverage tInputRA(3);

byte curDisplay = 1; //cycler for changing display readouts

elapsedMillis sinceLoop; //loop countup timer
elapsedMillis sinceDisplay;
elapsedMillis sinceUpdate;

int curCurrent = 1;
float peak1 = 1;
float peak2 = 1;
int peakBat = 1 ;
int loopPeak = 1;
//int coulombcount;

CRGB leds[NUM_LEDS];

//sabertooth setup

USBSabertoothSerial C;
USBSabertooth ST(C, 128);

//joystick setup



void setup() {

  display.begin(SSD1306_SWITCHCAPVCC);
  display.display(); // show splashscreen

  pinMode(xIn, INPUT);
  pinMode(yIn, INPUT);
  pinMode(batSensor, INPUT);
  pinMode(eStop, INPUT);

  FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(leds, NUM_LEDS);

  SabertoothTXPinSerial.begin(38400);
  ST.setTimeout(2000);  //safety time-out, best to make sure the commands are coming in faster than this

  batteryRA.clear();
  motor1RA.clear();
  motor2RA.clear();
  dInputRA.clear();
  tInputRA.clear();
  
  
  LEDStartup();

  if (abs(xCen-analogRead(xIn)) < 20 && abs(yCen-analogRead(yIn)) < 20){
    xCen = analogRead(xIn);
    yCen = analogRead(yIn);
  }
  display.clearDisplay();   // clears the screen and buffer
  sinceLoop = 0;
}

void loop() {
//update the inputs 
  dInputRA.addValue (constrain(analogRead(xIn) + (512 - xCen), 0, 1024));
  tInputRA.addValue (constrain(analogRead(yIn) + (512 - yCen), 0, 1024));

//check the estop state
  if (digitalRead(eStop) == 0 ) {
    eStopAct = 1; //the estop is on, better flag it
  }
  else //estop is off
  { 
    if (eStopAct == 1) //estop was previously on so better configure some things
    { 
      if (dInputRA.getAverage() > 800 && tInputRA.getAverage() > 800 ) //is the joystick in the bottom left corner?
      { 
        voyageur = true; //it is! turn on voyageur mode
        dStepLow = dStepLowV; //use the settings previously defined up above
        dStepHigh = dStepHighV;
        tStepLow = tStepLowV;
        tStepHighA = tStepHighAV;
        tStepHighB = tStepHighBV;
        dMax = dMaxV;
        tMax = dMaxV;
        dDecelRamp = dDecelRampV; //modifier for how much faster to decel 
        tDecelRamp =tDecelRampV;

        display.clearDisplay(); //clear the display and let the rider know that its now turbo
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(0, 0);
        display.println(F("Voyageur"));
        display.println(F("Mode"));
        display.println(F("Active"));
        display.display();
        while (abs(xCen-analogRead(xIn)) > 60 && abs(yCen-analogRead(yIn)) > 60){ //one last thing, best to wait for the joystick to centre before moving
          delay(2);
        }

      }
      else { //normal mode so configure accordingly
        voyageur = false;
        dStepLow = dStepLowN;
        dStepHigh = dStepHighN;
        tStepLow = tStepLowN;
        tStepHighA = tStepHighAN;
        tStepHighB = tStepHighBN;
        dMax = dMaxN;
        tMax = tMaxN;
        dDecelRamp = dDecelRampN; 
        tDecelRamp =tDecelRampN;
        if (abs(xCen-dInputRA.getAverage()) < 30 && abs(yCen-tInputRA.getAverage()) < 30){
          xCen =dInputRA.getAverage();
          yCen = tInputRA.getAverage(); //kludge to recentre the joystick at every release of the estop 
        }
      }

      eStopAct = 0; // okay its all set up, so lets reset the estop flag so it can run

    }
  }
  if (sinceUpdate > updateRate) { //no sense spamming things all the time
    updateMotor();
    updateCurrent();
    updateDisplay();
    sinceUpdate = 0;
  }


  // loopRA.addValue(sinceLoop);
  if (sinceLoop> loopPeak){ //basic timer to see how long its taking to run things
    loopPeak = sinceLoop;
  }
  sinceLoop = 0;
}

void LEDStartup() { //blinky light display on turn on makes sure its all working

  for (int counter = 0; counter <= 24; counter++) {
    for (int x = 0; x <= 24; x++)
    {
      if (x == counter) {
        leds[x] = CHSV( 0, 255, 50);
      }
      else {
        leds[x] = CHSV( 85, 255, 50);
      }

      if (counter > 24) {
        counter = 0;
      }
      FastLED.show();
      delay(1);
    }
  }

  for (int x = 0; x <= 24; x++)
  {
    leds[x] = CRGB::Black;
    FastLED.show();
  }
}


void updateMotor() {

  if (eStopAct == true) {//if the estop is on, set the input positions to 0 so it decelerates smoothly. 
    dInput = 0;
    tInput =0; 
  }
  else { //estop is off so pull and scale the values from the input average
    dInput = map(dInputRA.getAverage(), 0, 1024, 2047, -2047); 
    tInput = map(tInputRA.getAverage(), 0, 1024, -2047, 2047);
  }

  dStep = map(abs(dOutput - dInput),  0, 4094, dStepLow, dStepHigh); 
  //this line scales the step change by the difference between the desired position and the actual output, bigger difference faster change
  int scaleFac = map(abs(dOutput), 0, 2047, tStepHighA, tStepHighB); 
  //this line limits the rate of change on the turn axis in propotion to the magnitude of the drive axis
//tStep = map(abs(tOutput - tInput), 0, 4094, tStepLow, scaleFac);
tStep = map(abs(tInput), 0, 2047, tStepLow, scaleFac);

  if (dOutput < dInput) { //if the output is less than the input
    if (dOutput > 0) { //and the output is positive ie we are going forward
      dOutput = dOutput + dStep; //we are accelerating
    }
    else {
      dOutput = dOutput + dStep * dDecelRamp; //we are decelerating, lets do that decelramp times faster than we were accerating
    }
  }
  else if (dOutput > dInput) { //otherwise
    if (dOutput < 0) { //we are going backwards 
      dOutput = dOutput - dStep; //and accerlating
    }
    else {
      dOutput = dOutput - dStep * dDecelRamp; //or decelerating
    }

  }

//same stuff below as above, but for the turn axis, in this case its left and right rather than front or back but the same rules apply.
  if (tOutput < tInput) {
    if (tOutput > 0) { 
      tOutput = tOutput + tStep;
    }
    else {
      tOutput = tOutput + tStep * tDecelRamp;
    }
  }
  else if (tOutput > tInput) {
    if (tOutput <  0) {
      tOutput = tOutput - tStep;
    }
    else {
      tOutput = tOutput - tStep * tDecelRamp;
    }
  }

  //lets constrain things to the limits set above
  dOutput = constrain (dOutput, (-1 * dMax), dMax);
  tOutput = constrain (tOutput, (-1 * tMax), tMax);

//and send it on to the motor driver
  ST.drive(tOutput);
  ST.turn(dOutput);
}



void updateCurrent() {
  curCurrent = (analogRead(batSensor) - bMid) * bRatio; //read from the curent sensor and scale it appropriately
  // coulombcount = coulombcount +  (curCurrent / ((millis() - lastCycle) / 1000));
  float motor1Cur = float(ST.getCurrent(1, false)) / 10; //check in on the motors values returned are 0.1 amps, so divide by 10
  float motor2Cur = float(ST.getCurrent(2, false)) / 10;
  if (motor1Cur > peak1) {
    peak1 = motor1Cur;
  }
  if (motor2Cur > peak2) {
    peak2 = motor2Cur;
  }
  if (curCurrent > peakBat) {
    peakBat = curCurrent;
  }
  batteryRA.addValue(curCurrent);
  motor1RA.addValue(motor1Cur);
  motor2RA.addValue(motor2Cur);

}

void updateDisplay() {
  //update LEDs
  //current bar at top
  int curOut = map(batteryRA.getAverage(), -700, 700, -5, 5); 
  int m1Out =  map(motor1RA.getAverage(), -12, 12, -6, 6);
  int m2Out = map(motor2RA.getAverage(), -12, 12, -6, 6);


  for (int x = 0; x <= 5; x++)
  { 
    if (x < abs(curOut)) {
      if (curOut > 0) {
      leds[x] = CHSV( 0, 255, 50);
      leds[24 - x] = CHSV( 0, 255, 50);
      } 
      else {
        leds[x] = CHSV( 85, 255, 50);
       leds[25 - x] = CHSV( 85, 255, 50);
      }
    }
    else {
      leds[x] = CRGB::Black;     
      leds[25 - x] = CRGB::Black;
    }
  }



  int fade = cubicwave8(millis() / 4) / 5 + 30;
  if (ST.get('P', 1) > 0) {//checks the status of the brakes for motor 1
    leds[14] = CHSV( 85, 255, 50);
  } 
  else {
    leds[14] = CHSV( 0, 255, fade);
  }
  if (ST.get('P', 2) > 0) {//checks the status of the brakes for motor 2
    leds[12] = CHSV( 85, 255, 50);
  } 
  else {
    leds[12] = CHSV( 0, 255, fade);
  }

  for (int x = 1; x <= 6; x++)
  { 
    if (x < abs(m1Out)) {
      if (m1Out > 0) {
        leds[14 + x] = CHSV( 0, 255, 50);
      } 
      else {
        leds[14 + x] = CHSV( 85, 255, 50);
      }
    }
    else {
      leds[14 + x] = CRGB::Black;
    }
    if (x < abs(m2Out)) {
      if (m2Out > 0) {
        leds[12 - x] = CHSV( 0, 255, 50);
      } 
      else {
        leds[12 - x] = CHSV( 85, 255, 50);
      }
    }
    else {
      leds[12 - x] = CRGB::Black;
    }
  }

  if (eStopAct == false) {
    if (voyageur == true) {
      leds[13] = CHSV( 200, 200, 50);
    }
    else {
      leds[13] = CHSV( 85, 200, 50);
    }
  } 
  else {
    leds[13] = CHSV( 0, 200, fade);
  }

  FastLED.show();

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  switch (curDisplay) { //this cycles the display through the various readouts
  case 0:
    display.println(F("Inputs"));
    display.print(F("X: "));
    display.println( analogRead(xIn));
    display.print(F("Y: "));
    display.println( analogRead(yIn));
    break;
  case 1:
    display.println(F("Outputs"));
    display.print(F("D: "));
    display.println(dOutput);
    display.print(F("T: "));
    display.println(tOutput);
    break;
  case 2:
    display.println(F("Bat"));
    display.print(F("C: "));
    display.println(curCurrent);
    display.print(F("P: "));
    display.println(peakBat);
    break;
  case 3:
    display.println(F("Mot"));
    display.print(F("M1: "));
    display.println(peak1);
    display.print(F("M2: "));
    display.println(peak2);
    break;
  case 4:
    display.println(F("Loop"));
    display.print(F("since: "));
    display.println(sinceLoop);
    display.print(F("Peak: "));
    display.println(loopPeak);
    break;
  }
  display.display(); //print the display
  if (sinceDisplay > displayRate) { // has it been more than X seconds since we changed the display
    curDisplay++;
    if (curDisplay >4) //change this 
    {
      curDisplay =0;
    }
    sinceDisplay =0 ;
  }



}




