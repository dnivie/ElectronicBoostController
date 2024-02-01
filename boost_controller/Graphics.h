#ifndef GRAPHICS_H
#define GRAPHICS_H

#include "Arduino.h"

class Graphics
{
    public:
        uint8_t mapValToYpos(uint8_t val, uint8_t range, uint8_t y, uint8_t height);
        int16_t getSensorHistory(uint8_t index);
        void addSensorHistory(int16_t val);
        void init();
        uint16_t getBoostMax();
        int16_t getBoostMin();
        void resetBoostMax();
        void screenMode0(); // startup screen
        void screenMode1(float val0, float val1); // curved line graphics
        void screenMode2(float val0, float val1); // horizontal line graphics
        void screenMode3(float val0, float val1); // plotted boost curve
    
    private:
        void drawGraph(uint8_t x, uint8_t y, uint8_t len, uint8_t height);
        void drawHorizontalDotLine(uint8_t x, uint8_t y, uint8_t len);
        void drawGauge(uint8_t x, uint8_t y, uint8_t len, uint8_t maxHeight, uint16_t boostPressure);
        void drawAfrGraphics(uint8_t y, uint8_t height, float afr);
        void drawVerticalBar(uint8_t x, uint8_t y, uint8_t width, uint8_t maxHeight, uint16_t val);
        void drawBarGraph(int x, int y, int len, int height, int val);
};

#endif
