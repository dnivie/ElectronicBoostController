# Arduino Mega based boost controller, using a PID controlled solenoid to adjust boost pressure.

## A boost controller enhances the behavior of your turbo by increasing response and introduces safety-features against overboosting and lean fuel mixtures, which analog boost regulators lack. 

![alt text](https://cdn.shopify.com/s/files/1/1225/7944/products/3-Port_PWM_Diagram_grande.jpg?v=1462660855)

![alt text](https://3a663eb0fef48c6d2d60-a88f8ebfcdb877ad223e888bfcb7f7ec.ssl.cf1.rackcdn.com/861407_x600.jpg)

![alt text](https://cdn.shopify.com/s/files/1/0890/6136/products/defi-def-pdf00603s-18887449155_413x@3x.progressive.jpg)


### Parts used:
 - MAC 3-way solenoid
 - Arduino Mega. (UNO can be used but might bump into memory-issues because of the U8G2 library)
 - DEFI 3bar vacuum/boost pressure sensor (or any other 5V vacuum/air pressure sensor)
 - SSD1306 128x64 Oled screen (or any u8g2 compatible screen)
 - Bosch LSU4.2 Wideband Sensor


The MAC valves likes to operate at around 30Hz with PWM control. I have set the digital output d11 and d12 to 30Hz (see setup-function in code).

Here's the [link](https://a360.co/30CXK5K) to the gaugepod I'm using. It's design to be used with a 1.3" 128x64 oled screen with a 52mm standard automotive gaugepod.

### How to set up:
- Take the logging output (0-5V) from your wideband gauge into one of the analogIn arduino pins (A0, A1 etc)
- Boost sensor can be connected directly to arduino with power, ground and sensor output (assuming a 5V sensor is used)
- Solenoid output PWM-signal needs to be run with a mosfet, connected to 12V. Or you could buy a motor driver modul like the L298N.
- OLED screen is connected with SDL, SCL, PWR and GND directly to arduino.
- Make sure that the two wires on the solenoid are connected the right way, otherwise the solenoid will work with opposite effect.
