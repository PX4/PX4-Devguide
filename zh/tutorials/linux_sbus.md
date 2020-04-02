# Connecting an RC Receiver on Linux (Including S.Bus)

This topic shows how to setup a PX4 Linux-based autopilot to connect and use a [supported RC receiver](https://docs.px4.io/master/en/getting_started/rc_transmitter_receiver.html) on any serial port.

For RC types other than S.Bus, you can just connect the receiver directly to the serial ports, or to USB via a USB to TTY serial cable (e.g. like PL2302 USB to Serial TTL converter).

> **Note** For an S.Bus reciever (or encoder - e.g. from Futaba, RadioLink, etc.) you will usually need to connect the receiver and device via a [signal inverter circuit](#signal_inverter_circuit), but otherwise the setup is the same.

Then [Start the PX4 RC Driver](#start_driver) on the device, as shown below.

## Starting the Driver {#start_driver}

To start the RC driver on a particular UART (e.g. in this case `/dev/ttyS2`):

    rc_input start -d /dev/ttyS2
    

For other driver usage information see: [rc_input](../middleware/modules_driver.md#rcinput).

## Signal Inverter Circuit (S.Bus only) {#signal_inverter_circuit}

S.Bus is an *inverted* UART communication signal.

While some serial ports/flight controllers can read an inverted UART signal, most require a signal inverter circuit between the receiver and serial port to un-invert the signal.

> **Tip** This circuit is also required to read S.Bus remote control signals through the serial port or USB-to-TTY serial converter.

This section shows how to create an appropriate circuit.

### Required Components

* 1x NPN 晶体管（例如 NPN S9014 TO92）
* 1x 10K 电阻
* 1x 1K 电阻

> **Note** 可以使用任何类型/型号的晶体管，因为电流消耗非常低。

### Circuit Diagram/Connections

Connect the components as described below (and shown in the circuit diagram):

* S.Bus 信号&rarr;1K 电阻&rarr;NPN 晶体管
* NPN晶体管发射&rarr;GND
* 3.3VCC＆&rarr; 10K电阻&rarr; NPN晶体管集合&rarr; USB-to-TTY rxd
* 5.0VCC&rarr;S.Bus VCC
* GND &rarr; S.Bus GND

![Signal inverter circuit diagram](../../assets/driver_sbus_signal_inverter_circuit_diagram.png)

The image below shows the connections on a breadboard.

![Signal inverter breadboard](../../assets/driver_sbus_signal_inverter_breadboard.png)