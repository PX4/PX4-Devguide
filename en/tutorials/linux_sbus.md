# Sbus Driver for linux（drivers/linux_sbus）
This driver allows the autopilot to fetch data from the futaba sbus receiver via serial port, up to 16 channels.  
You can also use other receivers based on the sbus protocol, such as frsky, radioLink, and even sbus encoders.  
Onboard serial, USB to TTL serial cable/converter, and other types of serial ports are supported.  
All platforms running linux system are supported.  

## Required components

1x  NPN transistor  
1x  10K resistance  
1x  1K  resistor  

Any type of transistor can be used for this function. 

## Optional components
1x USB to TTL Serial Cable/Converter

## Installing
Connect the components according to the following ways  

S.bus singal->1K resistor->NPN transistor base  
NPN transistor emmit -> GND  
3.3v  VCC-> 10K resistor -> NPN transistor collection ->usbtty rxd  
5.0v  VCC->S.bus vcc  
GND ->S.bus gnd  

## Circuit diagram
![](http://www.playuav.com/uploads/article/20160310/56cf0f65bb1f7437c1618041a30dc308.png)
