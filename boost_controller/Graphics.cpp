#include "Graphics.h"
#include <U8g2lib.h>
#include <Arduino.h>

uint16_t boostMax = 0;
int16_t boostMin = 0;
const uint8_t sensorHistoryLength = 128;
uint8_t sensorHistory[sensorHistoryLength];
uint8_t sensorHistoryPos = sensorHistoryLength - 1;

//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);  // 1.3" screen
//U8G2_SSD1309_128X64_NONAME0_1_4W_SW_SPI u8g2(U8G2_R0, 13, 11, 10, 9, 8);    // 2.4" screen
U8G2_SSD1309_128X64_NONAME0_1_4W_SW_SPI u8g2(U8G2_R0, 13, 51, 10, 9, 8);    // 1.5" screen
// UNO/Mega: scl 13 green, sda 11, res 8 grey, dc 9 purple, cs 10 blue, mosi 51 yellow

void Graphics::init()
{
    u8g2.begin();
}

void Graphics::drawHorizontalDotLine(uint8_t x, uint8_t y, uint8_t len) 
{
    for (uint8_t i = 0; i < len; i++) 
    {
        if (!(i % 4)) u8g2.drawPixel(x + i, y);
    }
}

void Graphics::drawGraph(uint8_t x, uint8_t y, uint8_t len, uint8_t height)
{
    uint16_t absMin = abs(boostMin);    
    if(boostMax < 500)
    {
        boostMax = 500;
    }
    uint16_t range = absMin + boostMax; 
    // Draw 0 line
    //uint8_t zeroYPos = Graphics::mapValToYPos(absMin, range, y, height);
    Graphics::drawHorizontalDotLine(x, y, len);    
    // Draw the graph line
    for (uint8_t i = 0; i < 128; i++) 
    {
        // Scale the values so that the min is always 0
        uint8_t valueY = Graphics::getSensorHistory(i) + absMin;
    
        // Calculate the coordinants
        uint8_t yPos = Graphics::mapValToYpos(valueY, range, y, height);
        uint8_t xPos = len - i;
        u8g2.drawPixel(xPos, yPos);
        u8g2.drawPixel(xPos, yPos+1); 
        /*
        if (yPos < zeroYPos) 
        {
            // Point is above zero line, fill in space under graph
            u8g2.drawVLine(xPos, yPos, zeroYPos + 1 - yPos);
            //u8g2.drawPixel(xPos, yPos);
        } else 
        {
        // Point is below zero line, draw graph line without filling in
            u8g2.drawPixel(xPos, yPos);
        }
        */
    }
}

uint8_t Graphics::mapValToYpos(uint8_t val, uint8_t range, uint8_t y, uint8_t height)
{
    float valueY = ((float)val / range) * height;
    return y + height - (int)valueY;
}

int16_t Graphics::getSensorHistory(uint8_t index)
{
    index += sensorHistoryPos;
    if (index >= sensorHistoryLength) index = index - sensorHistoryLength;
    return sensorHistory[index];
}

void Graphics::addSensorHistory(int16_t val)
{
    sensorHistory[sensorHistoryPos] = val;
    sensorHistoryPos--;
    if (sensorHistoryPos < 0) sensorHistoryPos = sensorHistoryLength - 1;

    if (val > boostMax) boostMax = val;
    if (val < boostMin) boostMin = val;
}


uint16_t Graphics::getBoostMax()
{
    return boostMax;
}


int16_t Graphics::getBoostMin()
{
    return boostMin;
}


void Graphics::resetBoostMax()
{
    boostMax = 0;
}

// bar graphics (50% vacuum, 50% positive presure on a horizontal line)
void Graphics::drawBarGraph(int x, int y, int len, int height, int val) 
{
  uint16_t peakX = 1000;
  if(boostMax > peakX) // if boostPressure exceeds preset value, change graphics to compensate
  {
    peakX = boostMax;
  }
  // Draw the pressure bar behind the graph
  float barLen = (float(val) + peakX) / 1909.0;
  float barLength = barLen * len;

  u8g2.setDrawColor(2);
  u8g2.drawBox(x, y, barLength, height);
  u8g2.drawBox(64, y, 1, 12);
  u8g2.drawBox(x, y-1, 128, 1);
}


void Graphics::drawVerticalBar(uint8_t x, uint8_t y, uint8_t width, uint8_t maxHeight, uint16_t val)
{
  uint16_t top = 1000;
  float barValue = 0;
  barValue = abs(val);
  
  if(val >= 0)
  {
    barValue = 0;
  }

  //float barHeight = (float(val) + peakX) / 1909.0;
  float barHeight = (barValue / 1000) * maxHeight;
  u8g2.setDrawColor(2);
  u8g2.drawBox(x, y, width, barHeight);
}


void Graphics::drawAfrGraphics(uint8_t y, uint8_t height, float afr)
{
  float afrNormal = (afr / 10) - 1; // maps afr to [0,1]
  float x = afrNormal * 93; // multiply by horizontal screen length

  u8g2.setDrawColor(2);
  u8g2.drawBox(x-1, y, 1, height);
  u8g2.drawBox(x+1, y, 1, height);
  //u8g2.setFont(u8g2_font_7x13B_tf);
  //u8g2.drawStr(1, 24, "rich");
  //u8g2.drawStr(101, 24, "lean");
}

// draws some fancy line graphics
void Graphics::drawGauge(uint8_t x, uint8_t y, uint8_t len, uint8_t maxHeight, uint16_t boostPressure) 
{
  uint8_t barLength = (float(boostPressure)/1000) * len;
  uint8_t h = 0;
  uint8_t width = 5;
  for(uint8_t i = 0; i <= barLength; i+=8)
  {
    x = i;
    h = 1+i/8;
    y = (y + h*0.28)-4; 
    u8g2.setDrawColor(3);
    u8g2.drawBox(x,y,width,h);
  }
}


void Graphics::screenMode0()
{
    u8g2.firstPage();
    do
    {
        char cstr[6];
        u8g2.setFont(u8g2_font_mozart_nbp_h_all);
        u8g2.drawStr(13, 50, "$ bonsoir, Elliot");

    } while ( u8g2.nextPage() );
}


void Graphics::screenMode1(float boostPressure, float afr)
{
    u8g2.firstPage();
    do 
    {
        // Draw current pressure
        u8g2.setFont(u8g2_font_fub20_tf);
        char cstr[6];
        dtostrf((float)boostPressure/1000, 1, 2, cstr);
        u8g2.drawStr(0, 39, cstr);
        // Draw peak pressure
        u8g2.setFont(u8g2_font_fub11_tf);
        dtostrf((float)Graphics::getBoostMax() / 1000, 1, 2, cstr);
        uint16_t yPos = u8g2.getStrWidth(cstr);
        u8g2.drawStr(128 - yPos, 64, cstr);
        //writing
        u8g2.setFont(u8g2_font_fub11_tf);
        //u8g2.drawStr(103, 64, "bar");
        u8g2.drawStr(98, 14, "AFR");
        u8g2.drawStr(82, 29, "Turbo");
        u8g2.drawStr(64, 64, "bar");
        //Draw afr
        u8g2.setFont(u8g2_font_fub11_tf);
        dtostrf((float)afr, 1, 2, cstr);
        u8g2.drawStr(0, 14, cstr);

        u8g2.setDrawColor(2);
        u8g2.drawBox(0, 15, 128, 1);
        //drawBarGraph(0, 52, 128, 12, boostPressure);
        Graphics::drawGauge(0, 66, 128, 12, boostPressure);

    } while ( u8g2.nextPage() );
}


void Graphics::screenMode2(float boostPressure, float afr)
{
    u8g2.firstPage();
    do
    {
        //u8g2.setFont(u8g2_font_fub20_tf);
        u8g2.setFont(u8g2_font_fub25_tn);
        char cstr[6];
        dtostrf((float)boostPressure/1000, 1, 2, cstr);
        u8g2.drawStr(15, 54, cstr);
        // Draw peak pressure
        u8g2.setFont(u8g2_font_fub11_tf);
        dtostrf((float)Graphics::getBoostMax() / 1000, 1, 2, cstr);
        uint16_t yPos = u8g2.getStrWidth(cstr);
        u8g2.drawStr(95, 54, cstr);
        //writing
        //u8g2.setFont(u8g2_font_7x13B_tf);
        u8g2.setFont(u8g2_font_fub11_tf);
        //Draw afr
        dtostrf((float)afr, 1, 2, cstr);
        u8g2.drawStr(43, 24, cstr);
        
        Graphics::drawAfrGraphics(0, 10, afr);
        u8g2.drawBox(0, 10, 128, 1);
        //Serial.println(boostPressure);
        u8g2.setDrawColor(2);
        u8g2.drawBox(0, 25, 128, 1);
        Graphics::drawBarGraph(0, 57, 128, 7, boostPressure);

    } while ( u8g2.nextPage() );
}


void Graphics::screenMode3(float boostPressure, float afr)
{
    u8g2.firstPage();
    do
    {
        u8g2.setFont(u8g2_font_fub20_tf);
        char cstr[6];
        dtostrf((float)boostPressure / 1000, 1, 2, cstr);
        uint16_t yPos = u8g2.getStrWidth(cstr);
        u8g2.drawStr(128 - yPos, 32, cstr);
        //drawVerticalBar(123, 33, 5, 31, boostPressure);
        // draw max pressure
        u8g2.setFont(u8g2_font_mozart_nbp_h_all);
        dtostrf((float)Graphics::getBoostMax() / 1000, 1, 2, cstr);
        u8g2.drawStr(25, 32, cstr);
        //Draw afr
        u8g2.setFont(u8g2_font_mozart_nbp_h_all);
        u8g2.drawStr(0, 20, "boost");
        u8g2.drawStr(0, 32, "max:");
        dtostrf((float)afr, 1, 2, cstr);
        u8g2.drawStr(97, 9, cstr);
        Graphics::drawAfrGraphics(0, 10, afr);
        //u8g2.drawBox(0, 10, 128, 1);
        Graphics::drawHorizontalDotLine(0, 10, 128);
        // plotting
        Graphics::drawGraph(0, 33, 128, 31);

    } while ( u8g2.nextPage() );
}
