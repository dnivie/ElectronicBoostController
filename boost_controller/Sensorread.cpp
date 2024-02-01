#include "Sensorread.h"
#include "Kalman.h"
#define boostPin A0
#define afrPin A1

Sensor::Sensor() {}

float Sensor::readAfr(void)
{
  float afrData;
  afrData = Sensor::calculateAfr(analogRead(afrPin));
  return afrData;
}


float Sensor::calculateAfr(uint16_t n)
{
  // 10-20afr
  return ((n * (5.0/1023.0)) * 2) + 10;
}


float Sensor::readBoost(void)
{
  float boostData, boostPressure;
  boostData = Sensor::calculateBoost(analogRead(boostPin));
  // Sensor should read 0 at athmospheric pressure (tuned with sensor in car):
  boostPressure = boostData - 900;

  return boostPressure;
}

float Sensor::calculateBoost(uint16_t m)
{
  /*
  Scale the sensor reading into range
  https://stats.stackexchange.com/a/281164
  m = measurement to be scaled
  rmin = minimum of the range of the measurement
  rmax = maximum of the range of the measurement
  tmin = minimum of the range of the desired target scaling
  tmax = maximum of the range of the desired target scaling

  normalisedValue = ((m − rmin) / (rmax − rmin)) * (tmax − tmin) + tmin
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
