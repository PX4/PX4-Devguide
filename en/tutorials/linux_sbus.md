# SBus Driver for Linux
This driver allows the autopilot to fetch data from the futaba sbus receiver via serial port, up to 16 channels.  
You can also use other receivers based on the sbus protocol, such as FrSky, radioLink, and even sbus encoders.  
Onboard serial, USB to TTL serial cable/converter, and other types of serial ports are supported.  
All platforms running linux system are supported.  
`This driver is located in  Firm/src/drivers/linux_sbus`

## Required components

1x  NPN transistor  (I recommend NPN S9014 TO92)  
1x  10K resistor  
1x  1K  resistor  

`Because of the load current is small ,any type of transistor can be used for this purpose. `
`The devices described above are very common and are available anywhere, without regard to device models`


## Optional components
1x USB to TTL Serial Cable/Converter (I recommend PL2302 usb to tty serial converter)

`You can also use onboard serial port to instead`

## Installing
Connect the components according to the following  

S.bus singal->1K resistor->NPN transistor base  
NPN transistor emmit -> GND  
3.3VCC -> 10K resistor -> NPN transistor collection -> USB-to-TTY rxd
5.0VCC -> S.bus VCC  
GND -> S.bus GND  

## Circuit diagram  
S.bus is an inverted UART communication signal. Many flight controllers can read UART input, but cannot accept inverted one (such as the Naze32 Rev5) and an inverted is required. This circuit is an inverter, with this circuit, raspberry pi and anyother board can read S.bus remote control signal through the serial port or USB-to-TTY serial converter

![](http://www.playuav.com/uploads/article/20160310/56cf0f65bb1f7437c1618041a30dc308.png)

## Example
![](https://raw.githubusercontent.com/crossa/raspx4-sbus-rc-in/master/example.png)

## Launch it  
Add `linux_sbus start -d /dev/ttyUSB0 -c 8` to the configuration file and it will start automatically the driver on /dev/ttyUSB0 listening to 8 channels. The original configuration files are located in Firmware/posix-configs.   According to the official document, after you finish "make upload" related operations, all posix related configuration files will be placed in /home/pi, modify the file you want to use.

