#ifndef SOLENOID_H
#define SOLENOID_H

#include "Arduino.h"

class Solenoid
{
    public:
        void init();
        void open_loop(float val0, float val1);
        void closed_loop();
        void lean_check();
    private:
};

#endif