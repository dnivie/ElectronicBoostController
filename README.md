# Arduino Mega based boost controller, using a PID controlled solenoid to adjust boost pressure.

## A boost controller enhances the behavior of your turbo by increasing response and introduces safety-features against overboosting and lean fuel mixtures, which analog boost regulators lack. 

![alt text](https://cdn.shopify.com/s/files/1/1225/7944/products/3-Port_PWM_Diagram_grande.jpg?v=1462660855)

![alt text](https://3a663eb0fef48c6d2d60-a88f8ebfcdb877ad223e888bfcb7f7ec.ssl.cf1.rackcdn.com/861407_x600.jpg)


### Parts used:
 - MAC 3-way solenoid
 - Arduino Mega
 - DEFI 3bar vacuum/boost pressure sensor
 - SSD1306 128x64 Oled screen
 - Bosch LSU4.2 Wideband Sensor


The MAC valves likes to operate at around 30Hz with PWM control. I have set the digital output d11 and d12 to 30Hz (see setup-function in code).

