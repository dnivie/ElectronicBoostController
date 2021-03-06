#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>
#include <PID_v1.h>
#include "bitmap_gauge.h"

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);


int outPin = 11;  // output pin for solenoid
float boostPressure;
int pwmSignal;
String pwm_tx = "";
float afrNumber;

int boostMax = 0;
int boostMin = 0;
int buttonState = 0;

unsigned long startMillis;  //read sensor timer
unsigned long currentMillis;

unsigned long startPeakMillis; //peak reset timer
unsigned long currentPeakMillis;

const unsigned long period = 50;  //read sensor interval
const unsigned long peakPeriod = 15000; //peak boost will reset after 15 sec

double setpoint;
double input;
double output;

int kp = 2; //proportional
int ki = 0; //integral
int kd = 7; //derivative

PID myPID(&input, &output, &setpoint, kp, ki, kd, P_ON_M, DIRECT);
//P_ON_M specifies that Proportional on Measurement be used
//P_ON_E (Proportional on Error) is the default behavior

void setup() {
  // uno/nano:
  // set frequency on given port to 30Hz:
  //TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz (d9 & d10)
  //TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz (d3 & d11)

  // mega:
  // set frequency on given port to 30Hz:
  TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz (d11 & d12)
  //TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz (d9 & d10)

  u8g2.begin();
  pinMode(outPin, OUTPUT);
  Serial.begin(9600);
  startMillis = millis();
  startPeakMillis = millis();

  input = boostPressure;
  setpoint = 800;   //peak boost/1000 (bar)

  myPID.SetMode(AUTOMATIC);
}

void loop() {
  currentMillis = millis();
  currentPeakMillis = millis();

  if(currentMillis - startMillis >= period){
    readBoostData();
    readAfrSensor();
    //Serial.println(boostPressure);

    int pwm_out = solenoid_active();  //check if solenoid should activate
    //int pwm_out = solenoid_open_loop(); //manual pwm-control

    //transmitting to serial:
    //pwm_tx = String((pwm_out*100)/255);
    //pwm_tx += ',';
    //Serial.println(pwm_tx); //sends pwm-value as string for transmitting

    startMillis = currentMillis;
  }

  //display modes:
  switch (buttonState) {
    case 0: //all info display:
      u8g2.firstPage();
      do {
        draw(); //enable bitmap graphics (this makes the display slighty slower)
        
        // Draw current pressure
        u8g2.setFont(u8g2_font_fub20_tf);
        char cstr[6];
        dtostrf((float)boostPressure/1000, 1, 2, cstr);

        u8g2.drawStr(0, 38, cstr);

        // Draw peak pressure
        u8g2.setFont(u8g2_font_fub11_tf);
        dtostrf((float)boostMax / 1000, 1, 2, cstr);
        int yPos = u8g2.getStrWidth(cstr);
        u8g2.drawStr(128 - yPos, 11, cstr);

        //writing
        u8g2.setFont(u8g2_font_fub11_tf);
        u8g2.drawStr(96, 38, "Bar");
        //u8g2.drawStr(82, 40, "AFR");

        //boost controller signal
        u8g2.setFont(u8g2_font_fub11_tf);
        dtostrf((float)pwmSignal, 1, 1, cstr);
        u8g2.drawStr(85, 60, cstr);

        //draw afr
        u8g2.setFont(u8g2_font_fub11_tf);
        dtostrf((float)afrNumber, 1, 2, cstr);
        u8g2.drawStr(17, 61, cstr);
        

      } while ( u8g2.nextPage() );
      break;

    case 1: //boostpressure oriented display:
      u8g2.firstPage();
      do{
        u8g2.setFont(u8g2_font_fub25_tf);
        char cstr[6];
        dtostrf((float)boostPressure/1000, 1, 2, cstr);

        u8g2.drawStr(0, 30, cstr);

        // Draw peak pressure
        u8g2.setFont(u8g2_font_fub11_tf);
        dtostrf((float)boostMax / 1000, 1, 2, cstr);
        int yPos = u8g2.getStrWidth(cstr);
        u8g2.drawStr(128 - yPos, 11, cstr);

        //writing
        u8g2.setFont(u8g2_font_fub11_tf);
        u8g2.drawStr(82, 30, "Boost");
        u8g2.drawStr(82, 40, "AFR");

        //draw afr
        u8g2.setFont(u8g2_font_fub14_tf);
        dtostrf((float)afrNumber, 1, 2, cstr);
        u8g2.drawStr(0, 40, cstr);

      } while ( u8g2.nextPage() );
      break;
  }
}


void draw() { // draws bitmap to screen:
  u8g2.drawBitmap(0, 0, 16, 64, gaugegraphics);
}


// PID controlled solenoid
int solenoid_active(void){
  //Serial.println((pwmSignal*100)/255);

  if(boostPressure > 900 || (afrNumber > 15 && boostPressure > 700)){ // overboost or lean-protection. closes solenoid
      pwmSignal = 0;  //closes solenoid to minimize boostpressure.
      analogWrite(outPin, pwmSignal);
      //tone(6,1319,125); //warning note on pin 6
      delay(500);
      //noTone(6);
      //delay(500);
    }

    if(boostPressure > 100){  //start PID controller when boost > 0.5 bar
      setpoint = 800;   //target boost value
      Serial.println(pwmSignal);
      input = boostPressure;
      myPID.Compute();
      pwmSignal = output;
      analogWrite(outPin, pwmSignal);
    }
    else{
      pwmSignal = 255;
      Serial.println(pwmSignal);
      analogWrite(outPin, pwmSignal); //set solenoid to open which keeps wastegate closed. minimize turbolag.
    }
    return pwmSignal;
}

//manually controll pwm-signal
int solenoid_open_loop(void){
  int manPWM = analogRead(A2);  //potmeter
  pwmSignal = map(manPWM, 0, 1023, 0, 255);  //map to pwm-values
  
  if(boostPressure > 900){ //overboost protection. close solenoid
      pwmSignal = 255;
      analogWrite(outPin, pwmSignal);
      tone(6,1319,125); //warning tone
      delay(500);
      noTone(6);
      delay(500);
    }

    if (boostPressure > 600){
      analogWrite(outPin, pwmSignal);
    }
    else{
      analogWrite(outPin, 0);
    }
    return pwmSignal;  //return pwm-value for display
}


float normaliseSensorData(int m){
  /*
   *Scale the sensor reading into range
    m = measurement to be scaled
    rmin = minimum of the range of the measurement
    rmax = maximum of the range of the measurement
    tmin = minimum of the range of the desired target scaling
    tmax = maximum of the range of the desired target scaling
    normalisedValue = ((m − rmin) / (rmax − rmin)) * (tmax − tmin) + tmin
    */
    //my sensor (bar):
    //0.045 to 5v
    int rmin = 4;
    int rmax = 988;
    //Sensor reads from 0 to 2 bar
    int tmin = 0;
    int tmax = 2000;
    //normalisedValue = ((m - 9) / (1023 - 9)) * (4000 - 0) - 0

   //return (((m-rmin) / (rmax - rmin)) * (tmax - tmin) + tmin);
   return (m-9)/0.253;
}


void readBoostData(void){
  /*
  new boost sensor:
  pressure (bar): 0.2     0.2     0.4     0.5     0.6     0.7     0.8     0.9     1.0     2.0
  v:              0.273   0.526   0.779   1.032   1.285   1.538   1.791   2.044   2.297   4.827
  analog:         56      108     159     211     263     315     366     418     470     988
  */
  float absolutePressure = normaliseSensorData(analogRead(A0));

  //tune this value:
  boostPressure = absolutePressure - 300;

  // Update max and min
  if (boostPressure > boostMax) boostMax = boostPressure;
  if (boostPressure < boostMin) boostMin = boostPressure;

  if (currentPeakMillis - startPeakMillis > peakPeriod){ //reset peakValue;
      boostMax = 0;
      startPeakMillis = currentPeakMillis;
    }
}


float calculateAfrData(int n){
  //10-20afr
  return ((n * (5.0/1023.0)) * 2) + 10;
}


void readAfrSensor(void){
  float afrData = calculateAfrData(analogRead(A1));
  afrNumber = afrData;
}
