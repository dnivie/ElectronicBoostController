# Arduino Mega based boost controller, using a PID controlled solenoid to adjust boost pressure.

## A boost controller enhances the behavior of your turbo by increasing response and introducing safety features against overboosting, which analog boost regulators lack. 

### Parts used:
 - MAC 3-way solenoid
 - Arduino Mega
 - DEFI 3bar vacuum/boost pressure sensor
 - SSD1306 128x64 Oled screen
 - Bosch LSU4.2 Wideband Sensor


The MAC valves likes to operate at around 30Hz with PWM control. I have set the digital output d9 and d10 to 30Hz (see setup-function in code).

The boost-sensor being used is not ideal and I'm in still trying to find a more accurate sensor.
