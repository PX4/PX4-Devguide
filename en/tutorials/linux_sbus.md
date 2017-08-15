# Sbus Driver for linux（drivers/linux_sbus）
This driver allows the autopilot to fetch data from the futaba sbus receiver via serial port, up to 16 channels.  
Onboard serial, usbtty, and other types of serial ports are supported.  
All platforms running linux system are supported.  

## Required components

NPN transistor * 1  
10K resistance * 1  
1K resistor * 1  

## Optional components
Usbtty *1  

## Installation
Connect the components according to the following ways  

S.bus singal->1K resistor->NPN transistor base  
NPN transistor emmit -> GND  
3.3v  VCC-> 10K resistor -> NPN transistor collection ->usbtty rxd  
5.0v  VCC->S.bus vcc  
GND ->S.bus gnd  

## Circuit diagram
![](http://www.playuav.com/uploads/article/20160310/56cf0f65bb1f7437c1618041a30dc308.png)

## Launch the driver
After the following command started, the autopilot would fetch the sbus data via /dev/ttyUSB0, up to 8 channels.
```
pxh> linux_sbus start -d /dev/ttyUSB0 -c 8 
```




