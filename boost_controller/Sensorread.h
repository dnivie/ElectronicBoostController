#ifndef SENSORREAD_H
#define SENSORREAD_H

#include "Arduino.h"

class Sensor
{
  public:
    float calculateBoost(uint16_t boostPressure);
    float calculateAfr(uint16_t afr);
    Sensor();
    float readBoost(void);
    float readAfr(void);
};

#endif
