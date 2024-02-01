#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>
#include "Kalman.h"
#include "Sensorread.h"
#include "Graphics.h"
#include "Solenoid.h"

// https://playground.arduino.cc/Code/PIDLibraryAdaptiveTuningsExample/
// https://playground.arduino.cc/Code/PIDLibraryPonMExample/


unsigned long startMillis;  //read sensor timer
unsigned long currentMillis;
unsigned long startPeakMillis; //peak reset timer
unsigned long currentPeakMillis;

uint8_t screenMode = 0; // screen layout (see switch case below)
const uint8_t period = 50;  //read sensor interval
const uint16_t peakPeriod = 15000; //peak boost will reset after 15 sec
const uint16_t startUpPeriod = 1000;
uint8_t noiseCovariance = 60; // Kalman filtering

/*
const uint16_t setpoint = 800;   //peak boost/1000 (bar)
double input;
double output;
int kp = 2; //proportional
int ki = 0; //integral
int kd = 7; //derivative
int outPin = 11;  // output pin for solenoid
int pwmSignal;
*/

Kalman kf;
Sensor sensor;
Graphics screen;
Solenoid ebc;


void setup() {
  // uno/nano:
  // set frequency on given port to 30Hz:
  //TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of 30.64 Hz (d9 & d10)
  //TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of 30.64 Hz (d3 & d11)
  // mega:
  // set frequency on given port to 30Hz:
  TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of 30.64 Hz (d11 & d12)
  //TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of 30.64 Hz (d9 & d10)

  Serial.begin(9600);
  startMillis = millis();
  startPeakMillis = millis();
  kf.init(noiseCovariance);
  screen.init();
  ebc.init();
}


void loop(void) 
{  
  float boostPressure;
  float afr;
  //bool lean = false;
  currentMillis = millis();
  currentPeakMillis = millis();
  if (currentMillis - startMillis >= period)
  { 
    // read sensors and filter:
    boostPressure = sensor.readBoost();
    boostPressure = kf.filter(boostPressure);
    //lean = leanCheck(boostPressure, afrNumber);

    afr = sensor.readAfr();
    afr = kf.filter(afr);

    ebc.open_loop(boostPressure, afr);
    //int pwm_out = solenoid_active(boostPressure, lean);  // check if solenoid should activate (closed-loop)
    //int pwm_out = solenoid_open_loop(boostPressure, lean); // manual pwm-control (open-loop)

    if (currentPeakMillis - startPeakMillis > peakPeriod) // reset max boost pressure;
    {
      screen.resetBoostMax();
      startPeakMillis = currentPeakMillis;
    }

    screen.addSensorHistory(boostPressure);
    startMillis = currentMillis;
  }

  // display modes:
  switch (screenMode)
  {
    case 0:
      // startup screen
      screen.screenMode0();
      //unittest();

      if (currentMillis - startPeakMillis >= startUpPeriod)
      {
        screenMode = 1; // switch screen mode
      }
      
      break;

    case 1:
      screen.screenMode1(boostPressure, afr);
      
      break;
  }
}