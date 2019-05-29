# S.Bus Driver for Linux

The *S.Bus Driver for Linux* allows a Linux-based autopilot to access up to 16 channels from a *Futaba S.Bus receiver* via a serial port. The driver should also work with other receivers that use the S.Bus protocol, including as FrSky, RadioLink, and even S.Bus encoders.

A signal inverter circuit is required (described below) to enable the device serial port to read data from the receiver.

> **Note** The driver has been tested on Raspberry Pi running Rasbian Linux, when connected to the receiver through the onboard serial port or via a USB to TTY serial cable. It is expected to work on all Linux versions, and through all serial ports.

## Signal inverter circuit

S.Bus is an *inverted* UART communication signal. As many serial ports/flight controllers cannot read an inverted UART signal, a signal inverter circuit is required between the receiver and serial port un-invert the signal. This section shows how to create an appropriate circuit.

> **Tip** This circuit is required for Raspberry Pi to read S.Bus remote control signals through the serial port or USB-to-TTY serial converter. It will also be required for many other flight controllers.

### Required components

* 1x NPN transistor (e.g. NPN S9014 TO92) 
* 1x 10K resistor
* 1x 1K resistor

> **Note** Any type/model of transistor can be used because the current drain is very low.

<span></span>

> **Tip** Raspberry Pi only has a single serial port. If this is already being used you can alternatively connect your S.Bus receiver to the RaPi USB port, via a USB to TTY serial cable (e.g. PL2302 USB to TTL serial converter)

### Circuit diagram/Connections

Connect the components as described below (and shown in the circuit diagram):

* S.Bus signal &rarr; 1K resistor &rarr; NPN transistor base
* NPN transistor emit &rarr; GND
* 3.3VCC &rarr; 10K resistor &rarr; NPN transistor collection &rarr; USB-to-TTY rxd
* 5.0VCC &rarr; S.Bus VCC
* GND &rarr; S.Bus GND

![Signal inverter circuit diagram](../../assets/driver_sbus_signal_inverter_circuit_diagram.png)

### Breadboard image

The image below shows the connections on a breadboard.

![Signal inverter breadboard](../../assets/driver_sbus_signal_inverter_breadboard.png)

## Source code

* [Firmware/src/drivers/linux_sbus](https://github.com/PX4/Firmware/tree/master/src/drivers/linux_sbus)

## Usage

The command syntax is:

    linux_sbus start|stop|status -d <device> -c <channel>
    

So for example, to automatically start the driver listening to 8 channels on device `/dev/ttyUSB0`, you would add the following line to the startup configuration file.

    linux_sbus start -d /dev/ttyUSB0 -c 8
    

> **Note** The original configuration files are located in **Firmware/posix-configs**. According to the official documentation, after you finish `make upload` related operations, all posix related configuration files will be placed in **/home/pi**. You can modify the file you want to use there.