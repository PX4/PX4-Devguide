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

1. S.bus singal->1K resistor->NPN transistor base  
2. NPN transistor emmit GND  
3. 3VCC-> 10K resistor -> usbtty rxd
4



