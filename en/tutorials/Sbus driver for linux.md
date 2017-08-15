# Sbus Driver for linux（linux_sbus）
This driver allows the autopilot to read data from the futaba sbus receiver via serial port. 
Onboard serial, usbtty, and other types of serial ports can communicate with sbus receivers.
Raspberry pi and  all platforms running linux system is supportted.

## Required components

NPN transistor * 1  
10K resistance * 1  
1K resistor * 1  

## Optional components
Usbtty *1  

## Installation
Connect the components according to the following ways  

S.bus singal->1K resistor->NPN transistor base  
NPN transistor emmit GND  
3.3v  VCC-> 10K resistor -> usbtty rxd  
5.0v  VCC->S.bus vcc  
GND ->S.bus gnd  

## Circuit diagram
![](http://www.playuav.com/uploads/article/20160310/56cf0f65bb1f7437c1618041a30dc308.png)
