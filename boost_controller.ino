#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>
#include <PID_v1.h>
//#include "bitmap_gauge.h"

//U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R2); //type of  screen
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);  // 1.3" screen
//U8G2_SSD1309_128X64_NONAME0_1_4W_SW_SPI u8g2(U8G2_R0, 13, 11, 10, 9, 8);    // 2.4" screen
// UNO/Mega: scl 13, sda 11, res 8, dc 9, cs 10

// https://playground.arduino.cc/Code/PIDLibraryAdaptiveTuningsExample/
// https://playground.arduino.cc/Code/PIDLibraryPonMExample/


int outPin = 11;  // output pin for solenoid
float boostPressure;
int pwmSignal;
String pwm_tx = "";
float afrNumber;

int boostMax = 0;
int boostMin = 0;
int buttonState = 3;

unsigned long startMillis;  //read sensor timer
unsigned long currentMillis;

unsigned long startPeakMillis; //peak reset timer
unsigned long currentPeakMillis;

const unsigned long period = 50;  //read sensor interval
const unsigned long peakPeriod = 15000; //peak boost will reset after 15 sec
const unsigned long startUpPeriod = 1000;

const int sensorHistoryLength = 128;  //horizontal length of screen is 128 pixels
int sensorHistory[sensorHistoryLength];
int sensorHistoryPos = sensorHistoryLength - 1;

double setpoint = 800;   //peak boost/1000 (bar)
double input;
double output;

int kp = 2; //proportional
int ki = 0; //integral
int kd = 7; //derivative

PID myPID(&input, &output, &setpoint, kp, ki, kd, P_ON_E, DIRECT);
// P_ON_M specifies that Proportional on Measurement be used
// P_ON_E (Proportional on Error) is the default behavior

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
  //Serial.begin(9600);
  startMillis = millis();
  startPeakMillis = millis();

  input = boostPressure;

  myPID.SetMode(AUTOMATIC);
}

void loop() {
  //float boostPressure = 0;
  //float afrNumber = 0;
  bool lean = false;
  
  currentMillis = millis();
  currentPeakMillis = millis();

  if(currentMillis - startMillis >= period){
    boostPressure = readBoostData();
    afrNumber = readAfrSensor();
    lean = leanCheck(boostPressure, afrNumber);
    //Serial.println(boostPressure);
    //int pwm_out = solenoid_active(boostPressure, lean);  // check if solenoid should activate (closed-loop)
    int pwm_out = solenoid_open_loop(boostPressure, lean); // manual pwm-control (open-loop)

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
        //draw(); // enable bitmap graphics (this makes the display slighty slower)
        
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

    case 1: // boostpressure oriented graph display:
      u8g2.firstPage();
      do{
        // current pressure
        u8g2.setFont(u8g2_font_fub20_tf);
        char cstr[6];
        dtostrf((float)boostPressure / 1000, 1, 2, cstr);
        u8g2.drawStr(0, 20, cstr);

        // Draw max pressure
        u8g2.setFont(u8g2_font_fub11_tf);
        dtostrf((float)boostMax / 100, 1, 2, cstr);
        int yPos = u8g2.getStrWidth(cstr);
        u8g2.drawStr(128 - yPos, 11, cstr);

        // plotting
        drawGraph(0, 32, 128, 31);

      } while ( u8g2.nextPage() );
      break;

    case 2:
    // screen mode 3 (circular gauge)
    u8g2.firstPage();
    do {
        DrawCircularGauge(30,30,30,10,boostPressure,-1,1.5);
        
        char cstr[6];
        u8g2.setFont(u8g2_font_fub20_tf);
        dtostrf((float)boostPressure, 1, 1, cstr);
        u8g2.drawStr(48, 64, cstr);
        u8g2.setFont(u8g2_font_7x13B_tf);
        u8g2.drawStr(103, 64, "bar");   
    } while ( u8g2.nextPage() );
    break;

    case 3:
    // startup message
    u8g2.firstPage();
    do {
        
        u8g2.setFont(u8g2_font_fub11_tf);
        u8g2.drawStr(0, 35, "hello friend"); // bonsoir, Elliot 

      if (currentMillis - startPeakMillis >= startUpPeriod){
        buttonState = 0;
      }

    } while ( u8g2.nextPage() );
    break;
  }
}

/*
void draw() { // draws bitmap to screen:
  u8g2.drawBitmap(0, 0, 16, 64, gaugegraphics);
}
*/

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


float normaliseSensorData(int m) {  // calculate sensorValue
  // check: input voltage from arduino, raw value (engine off)
  /*
    Scale the sensor reading into range
    m = measurement to be scaled
    rmin = minimum of the range of the measurement
    rmax = maximum of the range of the measurement
    tmin = minimum of the range of the desired target scaling
    tmax = maximum of the range of the desired target scaling
    normalisedValue = ((m − rmin) / (rmax − rmin)) * (tmax − tmin) + tmin
    https://stats.stackexchange.com/a/281164
  */

  /*
    Sensor voltage ranges from 0.273 to 4.827v, converted to analogRead values (0 min, 1023 max) that's 56 to 988
    rmin = 56
    rmax = 988
    
    tmin = 20
    tmax = 2000
    normalisedValue = ((m − 56) / (988 − 56)) * (2000 − 20) + (20)
    normalisedValue = ((m − 56) / 932) * 2000
    normalisedValue = (m − 56) / 0.466

    normalisedValue = ((m - 9) / (921 - 9)) * (3000 - 0) - 0
  */
  return (m-56)/0.466;
}


float readBoostData(void) {
  //int testData = 580;
  //float absolutePressure = normaliseSensorData(testData);
  float absolutePressure = normaliseSensorData(analogRead(A0));
  absolutePressure = kalmanFilter(absolutePressure); // filter measurement 
  // Subtract 14.7psi/1bar (pressure at sea level)
  
  // tune this value until sensor shows 0 with the engine off:
  float boostPressure = absolutePressure - 900;

  // Update max and min
  if (boostPressure > boostMax){
    boostMax = boostPressure;
    startPeakMillis = currentPeakMillis; // reset timer when new boostMax is reached
  }

  if (currentPeakMillis - startPeakMillis > peakPeriod){ //reset peakValue;
      boostMax = 0;
      startPeakMillis = currentPeakMillis;
    }

  // Log the history:
  addSensorHistory(boostPressure);
  
  return boostPressure;
}

// check if fuel mixture is lean while producing boost pressure
bool leanCheck(float boost, float afr){
  bool leanMixture = false;
  
  if(boost > 500 && afr > 13){
    leanMixture = true;
  }

  return leanMixture;
}

float calculateAfrData(int n){
  //10-20afr
  return ((n * (5.0/1023.0)) * 2) + 10;
}

float kalmanFilter(float U){
  static const float R = 40; // noise covariance
  static const float H = 1; // measurement map scalar
  static float Q = 10; // initial estimated covariance
  static float P = 0; // initial error covariance (must be 0)
  static float uHat = 0; // initial estimated state
  static float K = 0; // initial kalman gain

  // update:
  K = P*H / (H*P*H+R); // update kalman gain
  uHat = uHat + K*(U-H*uHat); // update estimate

  // update error covariance:
  P = (1-K*H) * P+Q;

  // return estimate of uHat (the state):
  return uHat;
}


float readAfrSensor(void){
  float afrData = calculateAfrData(analogRead(A1));
  afrNumber = afrData;
  return afrNumber;
}

void addSensorHistory(int val) {
  sensorHistory[sensorHistoryPos] = val;
  sensorHistoryPos--;
  if (sensorHistoryPos < 0) sensorHistoryPos = sensorHistoryLength - 1;
}


int getSensorHistory(int index) {
  index += sensorHistoryPos;
  if (index >= sensorHistoryLength) index = index - sensorHistoryLength;
  return sensorHistory[index];
}

// Maps a value to a y height
int mapValueToYPos(int val, int range, int y, int height) {
  float valueY = ((float)val / range) * height;
  return y + height - (int)valueY;
}


void drawHorizontalDottedLine(int x, int y, int len) {
  for (int i = 0; i < len; i++) {
    if (!(i % 4)) u8g2.drawPixel(x + i, y);
  }
}

// circular gauge (3 quarter)
void DrawCircularGauge(int x, byte y, byte r, byte p, int value, int minVal, int maxVal) {
  // x,y = coordinate, r = radius, p = thickness
  int n=(r/100.00)*p; // calculates the length 
  // top half filled
  int ns;
  float gs_rad=-3.14; // start of circle 
  float ge_rad=1.572; // end 
  
  n=r-1; 
  ns=r-p;
  
  u8g2.drawCircle(x,y,r, U8G2_DRAW_LOWER_LEFT|U8G2_DRAW_UPPER_LEFT|U8G2_DRAW_UPPER_RIGHT); // scale outside
  u8g2.drawCircle(x,y,r-p, U8G2_DRAW_LOWER_LEFT|U8G2_DRAW_UPPER_LEFT|U8G2_DRAW_UPPER_RIGHT); // internal scale
  u8g2.drawLine(x-r, y, x-r+(p+2), y); // scale line at min
  u8g2.drawLine(x+r, y, x+r-(p+2), y); // scale line at max                   
  u8g2.drawLine(x, y-r, x, y-r+(p+2)); // half scale line
  u8g2.drawLine(x, (y+r), x, y+(r-p)-2); // bottom scale line      
  u8g2.drawCircle(x,y,1);
  
  float l=((value-minVal)*(ge_rad-gs_rad)/(maxVal-minVal)+gs_rad); // displaying a needle from center of circle
  int xp = x+(sin(l) * n);  // x end point of the line
  int yp = y-(cos(l) * n);  // y end point of the line
  int x1 = x;
  int y1 = y;
  u8g2.drawLine(x1,y1,xp,yp); // line to fill the scale
  
  for(int k=1; k<=value; k+=22) { // displaying a scale filled with many lines next to each other
    float i=((k-minVal)*(ge_rad-gs_rad)/(maxVal-minVal)+gs_rad);
    int xp = x+(sin(i) * n);  // x end point of the line
    int yp = y-(cos(i) * n);  // y end point of the line
    int xs = x+(sin(i) * ns); // x the start point of the lines
    int ys = y-(cos(i) * ns); // y the start point of the lines
    u8g2.drawLine(xs,ys,xp,yp); // line to fill the scale          
  }                 
}


void drawGraph(int x, int y, int len, int height) {
  // Draw the lines
  drawHorizontalDottedLine(x, y, len);
  drawHorizontalDottedLine(x, y + height, len);

  //var absMin = Math.abs(boostMin);
  int absMin = abs(boostMin);
  int range = absMin + boostMax;

  // Draw 0 line
  int zeroYPos = mapValueToYPos(absMin, range, y, height);
  drawHorizontalDottedLine(x, zeroYPos, len);

  // Draw the graph line
  for (int i = 0; i < 128; i++) {
    // Scale the values so that the min is always 0
    int valueY = getSensorHistory(i) + absMin;

    // Calculate the coordinants
    int yPos = mapValueToYPos(valueY, range, y, height);
    int xPos = len - i;
    if (yPos < zeroYPos) {
      // Point is above zero line, fill in space under graph
      u8g2.drawVLine(xPos, yPos, zeroYPos + 1 - yPos);
    } else {
      // Point is below zero line, draw graph line without filling in
      u8g2.drawPixel(xPos, yPos);
    }
  }
}
