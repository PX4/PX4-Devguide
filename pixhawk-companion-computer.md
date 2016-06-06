# Companion Computer for Pixhawk class

Interfacing a companion computer (Raspberry Pi, Odroid, Tegra K1) to Pixhawk-family boards always works the same way: They are interfaced using a serial port to `TELEM2`, the port intended for this purpose. The message format on this link is [MAVLink](http://mavlink.org).

## Pixhawk setup

Set the `SYS_COMPANION` parameter (in the System group) to one of these values.

<aside class="tip">
Changing this parameter requires an autopilot reboot to become active.
</aside>

  * `0` to disable MAVLink output on TELEM2 (default)
  * `921600` to enable MAVLink output at 921600 baud, 8N1 (recommended)
  * `157600` to enable MAVLink in *OSD* mode at 57600 baud
  * `257600` to enable MAVLink in listen-only mode at 57600 baud

## Companion computer setup

In order to receive MAVLink, the companion computer needs to run some software talking to the serial port. The most common options are:

  * [MAVROS](ros-mavros-installation.md) to communicate to ROS nodes
  * [C/C++ example code](https://github.com/mavlink/c_uart_interface_example) to connect custom code
  * [MAVProxy](http://mavproxy.org) to route MAVLink between serial and UDP

## Hardware setup

Wire the serial port according to the instructions below. All Pixhawk serial ports operate at 3.3V and are 5V level compatible.

<aside class="caution">
Many modern companion computers only support 1.8V levels on their hardware UART and can be damaged by 3.3V levels. Use a level shifter. In most cases the accessible hardware serial ports already have some function (modem or console) associated with them and need to be *reconfigured in Linux* before they can be used.
</aside>

The safe bet is to use an FTDI Chip USB-to-serial adapter board and the wiring below. This always works and is easy to set up.

| TELEM2 |         | FTDI    |        |
--- | --- | ---
|1         | +5V (red)|         | DO NOT CONNECT!   |
|2         | Tx  (out)| 5       | FTDI RX (yellow) (in)   |
|3         | Rx  (in) | 4       | FTDI TX (orange) (out)  |
|4         | CTS (in) |6       | FTDI RTS (green) (out) |
|5         | RTS (out)|2       | FTDI CTS (brown) (in) |
|6         | GND     | 1       | FTDI GND (black)   |
