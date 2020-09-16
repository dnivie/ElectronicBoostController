#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>
#include <PID_v1.h>

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

int outPin = 9;
float boostPressure;
int pwmSignal;
String pwm_tx = "";
float afrNumber;

int boostMax = 0;
int boostMin = 0;

unsigned long startMillis;  //read sensor timer
unsigned long currentMillis;

unsigned long startPeakMillis; //peak reset timer
unsigned long currentPeakMillis;

const unsigned long period = 50;  //read sensor interval
const unsigned long peakPeriod = 15000; //peak boost will reset after 15 sec

double setpoint;
double input;
double output;


int kp = 0.3; //proportional
int ki = 0; //integral
int kd = 0.1; //derivative

PID myPID(&input, &output, &setpoint, kp, ki, kd, P_ON_M, DIRECT);
//P_ON_M specifies that Proportional on Measurement be used
//P_ON_E (Proportional on Error) is the default behavior

void setup() {
  // uno/nano:
  // set frequency on given port to 30Hz:
  TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz (d9 & d10)
  //TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz (d3 & d11)

  // mega:
  // set frequency on given port to 30Hz:
  //TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz (d11 & d12)
  //TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz (d9 & d10)

  u8g2.begin();
  pinMode(outPin, OUTPUT);
  Serial.begin(9600);
  startMillis = millis();
  startPeakMillis = millis();

  input = analogRead(A1); //boost pressure
  setpoint = 800;   //peak boost/1000 (bar)

  myPID.SetMode(AUTOMATIC);
}

void loop() {
  currentMillis = millis();
  currentPeakMillis = millis();
  readAfrSensor();

  if(currentMillis - startMillis >= period){
    readBoostData();
    //Serial.println(boostPressure);

    //int pwm_out = solenoid_active();  //(closed loop) check if solenoid should activate
    int pwm_out = solenoid_open_loop(); //(open loop) manual pwm-control
    
    //transmitting to serial:
    pwm_tx = String((pwm_out*100)/255);
    pwm_tx += ',';
    Serial.println(pwm_tx); //sends pwm-value as string for transmitting

    startMillis = currentMillis;
  }

  u8g2.firstPage();
  do {
    // Draw current pressure
    u8g2.setFont(u8g2_font_fub20_tf);
    char cstr[6];
    dtostrf((float)boostPressure/1000, 1, 2, cstr);

    u8g2.drawStr(0, 20, cstr);

    // Draw peak pressure
    u8g2.setFont(u8g2_font_fub11_tf);
    dtostrf((float)boostMax / 1000, 1, 2, cstr);
    int yPos = u8g2.getStrWidth(cstr);
    u8g2.drawStr(128 - yPos, 11, cstr);

    //writing
    u8g2.setFont(u8g2_font_fub11_tf);
    u8g2.drawStr(82, 23, "Boost");
    u8g2.drawStr(82, 40, "AFR");

    //boost controller behavior: (shows pwm output)
    u8g2.setFont(u8g2_font_fub11_tf);
    dtostrf((float)pwmSignal, 1, 2, cstr);
    u8g2.drawStr(0, 60, cstr);

    if (pwmSignal == 255){
      u8g2.setFont(u8g2_font_fub11_tf);
      u8g2.drawStr(0, 60, "Closed");
    }
    else if(pwmSignal == 0){
      u8g2.setFont(u8g2_font_fub11_tf);
      u8g2.drawStr(0, 60, "Open");
    }

    //draw afr
    u8g2.setFont(u8g2_font_fub14_tf);
    dtostrf((float)afrNumber, 1, 2, cstr);
    u8g2.drawStr(0, 40, cstr);


  } while ( u8g2.nextPage() );

}


int solenoid_active(void){
  //Serial.println((pwmSignal*100)/255);

  if(boostPressure > 900){ //overboost protection. closes solenoid
      pwmSignal = 255;
      analogWrite(outPin, pwmSignal);
      tone(6,1319,125); //warning tone
      delay(500);
      noTone(6);
      delay(500);
    }

    if(boostPressure > 600){  //start PID controller
      setpoint = 800;   //target boost value/1000 (bar)
      input = boostPressure;
      myPID.Compute();
      pwmSignal = output;
      analogWrite(outPin, pwmSignal);
    }
    else{
      pwmSignal = 0;
      analogWrite(outPin, pwmSignal); //set regulator to open. minimize turbolag.
    }
    return pwmSignal;
}

//manually controll pwm-signal
int solenoid_open_loop(void){
  int manPWM = analogRead(A2);  //potmeter
  pwmSignal = map(manPwm, 0, 1023, 0, 255);  //map to pwm-values
  if(boostPressure > 900){ //overboost protection. close solenoid
      pwmSignal = 255;
      analogWrite(outPin, pwmSignal);
      tone(6,1319,125); //warning tone
      delay(500);
      noTone(6);
      delay(500);
    }

    //overboost protection
    if (boostPressure > 600){
      analogWrite(outpin, pwmSignal);
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
    int rmin = 9;
    int rmax = 921;
    //Sensor reads from -1 to 3 bar
    int tmin = 0;
    int tmax = 3000;
    //normalisedValue = ((m - 9) / (1023 - 9)) * (4000 - 0) - 0

   return (((m-rmin) / (rmax - rmin)) * (tmax - tmin) + tmin);
   //return (m-9)/0.253;
}


void readBoostData(void){
  float absolutePressure = normaliseSensorData(analogRead(A1));

  //tune this value:
  boostPressure = absolutePressure - 400;

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
  float afrData = calculateAfrData(analogRead(A0));
  afrNumber = afrData;
}
