#include "Solenoid.h"
#include <Arduino.h>
#include <PID_v1.h>


const uint16_t setpoint = 800;   //peak boost/1000 (bar)
float input;
float output;
uint8_t kp = 10; //proportional gain
uint8_t ki = 0; //integral gain
uint8_t kd = 0.5; //derivative gain
uint8_t outPin = 11;  // output pin for solenoid
uint16_t pwmSignal;

void Solenoid::init()
{
    input = boostPressure;
    myPID.SetMode(AUTOMATIC);
    pinMode(outPin, OUTPUT);
    //PID myPID(&input, &output, &setpoint, kp, ki, kd, P_ON_E, DIRECT);
    // P_ON_M specifies that Proportional on Measurement be used
    // P_ON_E (Proportional on Error) is the default behavior
}

void Solenoid::open_loop(float boostPressure, float afr)
{
    // if building boost pressure -> give pwm signal to solenoid
    // if boostpressure above max -> 100% pwm signal to solenoid
    // create failsafe when high afr value
}

void Solenoid::closed_loop(){}

void Solenoid::lean_check(){}

/*
// PID controlled solenoid
int solenoid_active(float boostPressure, bool lean){
  //Serial.println((pwmSignal*100)/255);
  if(boostPressure > 100){  //start PID controller 
      //Serial.println(pwmSignal);
      input = boostPressure;
      myPID.Compute();
      pwmSignal = output;
      analogWrite(outPin, pwmSignal);
      
      if(boostPressure > 900 || lean == true){  // overboost or lean-protection.
        pwmSignal = 0;  // closes solenoid to minimize boostpressure.
        analogWrite(outPin, pwmSignal);
        //tone(6,1319,125); // warning note on pin 6
        delay(500);
        //noTone(6);
      }
    }
    else{
      pwmSignal = 255;   // open
      //Serial.println(pwmSignal);
      analogWrite(outPin, pwmSignal); // set solenoid to open which keeps wastegate closed. minimize turbolag.
    }
  return pwmSignal;
}


//manually controll pwm-signal
int solenoid_open_loop(float boostPressure, bool lean){
  //int manPWM = analogRead(A2);  //potmeter
  int manPWM = 200; // 0-1023 (0-100%) do some testing
  pwmSignal = map(manPWM, 0, 1023, 0, 255);  // map to pwm-values
  
  if(boostPressure > 800 || lean == true){ // overboost protection. close solenoid
      pwmSignal = 255;
      analogWrite(outPin, pwmSignal);
      //tone(6,1319,125); // warning tone
      delay(500);
      //noTone(6);
      //delay(500);
    }

    if (boostPressure > 100){ // start sending PWM signal to solenoid 
      analogWrite(outPin, pwmSignal);
    }
    else{
      analogWrite(outPin, 0); // set solenoid to open
    }
    return pwmSignal;  // return pwm-value for display
}



// check if fuel mixture is lean while producing boost pressure
bool leanCheck(float boost, float afr){
  bool leanMixture = false;
  
  if(boost > 500 && afr > 13){
    leanMixture = true;
  }

  return leanMixture;
}
*/