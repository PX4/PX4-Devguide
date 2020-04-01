# Connecting an S.Bus Receiver on Linux

This topic shows how to setup a linux-based autopilot to use an S.Bus reciever (or encoder - e.g. from Futaba, RadioLink, etc.) via any serial port.

The main requirements are:
- A [signal inverter circuit](#signal_inverter_circuit) is (usually) needed to connect the receiver and device.
- [Start the generic PX4 RC driver](#start_driver) on the device.

> **Note** The approach is expected to work for all Linux versions and through all serial ports, including via a USB to TTY serial cable (e.g. like PL2302 USB to Serial TTL converter).

## Signal Inverter Circuit {#signal_inverter_circuit}

S.Bus is an *inverted* UART communication signal.

While some serial ports/flight controllers can read an inverted UART signal, most require a signal inverter circuit between the receiver and serial port to un-invert the signal.

> **Tip** This circuit is also required to read S.Bus remote control signals through the serial port or USB-to-TTY serial converter.

This section shows how to create an appropriate circuit.

### Required Components

* 1x NPN transistor (e.g. NPN S9014 TO92)
* 1x 10K resistor
* 1x 1K resistor

> **Note** Any type/model of transistor can be used because the current drain is very low.


### Circuit Diagram/Connections

Connect the components as described below (and shown in the circuit diagram):

* S.Bus signal &rarr; 1K resistor &rarr; NPN transistor base
* NPN transistor emit &rarr; GND
* 3.3VCC &rarr; 10K resistor &rarr; NPN transistor collection &rarr; USB-to-TTY rxd
* 5.0VCC &rarr; S.Bus VCC
* GND &rarr; S.Bus GND

![Signal inverter circuit diagram](../../assets/driver_sbus_signal_inverter_circuit_diagram.png)

The image below shows the connections on a breadboard.

![Signal inverter breadboard](../../assets/driver_sbus_signal_inverter_breadboard.png)

## Starting the Driver {#start_driver}

To start the RC driver on a particular UART (e.g. in this case `/dev/ttyS2`): 
```
rc_input start -d /dev/ttyS2
```

For other driver usage information see: [rc_input](../middleware/modules_driver.md#rcinput).
