# Companion Computer for Pixhawk Series

Pixhawk与配套计算机(Raspberry Pi，Odroid，Tegra K1) 的交互方式只有一种：通过串口2 `TELEM 2`。这个串口设计目的即是此。 消息格式是MAVLINK。

## Pixhawk设置

在 任何 [可配置的串口 ](https://docs.px4.io/en/peripherals/serial_configuration.html)上使能MAVLink消息。

> **提示** `TELEM 2` 一般用作与配套计算机通信。

要在`TELEM 2` 上配置默认的配套计算机消息流，请设置以下参数：

* [MAV_1_CONFIG](../advanced/parameter_reference.md#MAV_1_CONFIG) = `TELEM 2` (`MAV_1_CONFIG`总是配置为 `TELEM 2` 端口)
* [MAV_1_MODE](../advanced/parameter_reference.md#MAV_1_MODE) = `Onboard`
* [MAV_X_RATE](../advanced/parameter_reference.md#MAV_X_RATE) = `921600` (或者更高)

更多信息，请参考 [MAVLink Peripherals (GCS/OSD/Companion)](https://docs.px4.io/en/peripherals/mavlink_peripherals.html)。

## 配套计算机设置

为了接收 mavlink消息, 配套计算机需要运行一些与串行端口对话的软件。 最常见的选择有:

* [MAVROS](../ros/mavros_installation.md) 与ros 节点通信
* C/C++ example code </0 > 连接自定义代码</li> 
    
    * [MAVProxy](http://mavproxy.org) 在串行和 udp 之间转换 mavlink</ul> 
    
    ## Hardware setup
    
    Wire the serial port according to the instructions below. All Pixhawk serial ports operate at 3.3V and are 5V level compatible.
    
    > **Warning** Many modern companion computers only support 1.8V levels on their hardware UART and can be damaged by 3.3V levels. Use a level shifter. In most cases the accessible hardware serial ports already have some function (modem or console) associated with them and need to be *reconfigured in Linux* before they can be used.
    
    The safe bet is to use an FTDI Chip USB-to-serial adapter board and the wiring below. This always works and is easy to set up.
    
    |  | TELEM2 |           | FTDI |                        |
    |  | ------ | --------- | ---- | ---------------------- |
    |  | 1      | +5V (red) |      | DO NOT CONNECT!        |
    |  | 2      | Tx (out)  | 5    | FTDI RX (yellow) (in)  |
    |  | 3      | Rx (in)   | 4    | FTDI TX (orange) (out) |
    |  | 4      | CTS (in)  | 6    | FTDI RTS (green) (out) |
    |  | 5      | RTS (out) | 2    | FTDI CTS (brown) (in)  |
    |  | 6      | GND       | 1    | FTDI GND (black)       |
    
    ## Software setup on Linux
    
    On Linux the default name of a USB FTDI would be like `\dev\ttyUSB0`. If you have a second FTDI linked on the USB or an Arduino, it will registered as `\dev\ttyUSB1`. To avoid the confusion between the first plugged and the second plugged, we recommend you to create a symlink from `ttyUSBx` to a friendly name, depending on the Vendor and Product ID of the USB device.
    
    Using `lsusb` we can get the vendor and product IDs.
    
    ```sh
    $ lsusb
    
    Bus 006 Device 002: ID 0bda:8153 Realtek Semiconductor Corp.
    Bus 006 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
    Bus 005 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
    Bus 004 Device 002: ID 05e3:0616 Genesys Logic, Inc.
    Bus 004 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
    Bus 003 Device 004: ID 2341:0042 Arduino SA Mega 2560 R3 (CDC ACM)
    Bus 003 Device 005: ID 26ac:0011
    Bus 003 Device 002: ID 05e3:0610 Genesys Logic, Inc. 4-port hub
    Bus 003 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
    Bus 002 Device 001: ID 1d6b:0001 Linux Foundation 1.1 root hub
    Bus 001 Device 002: ID 0bda:8176 Realtek Semiconductor Corp. RTL8188CUS 802.11n WLAN Adapter
    Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
    ```
    
    The Arduino is `Bus 003 Device 004: ID 2341:0042 Arduino SA Mega 2560 R3 (CDC ACM)`
    
    The Pixhawk is `Bus 003 Device 005: ID 26ac:0011`
    
    > **Note** If you do not find your device, unplug it, execute `lsusb`, plug it, execute `lsusb` again and see the added device.
    
    Therefore, we can create a new UDEV rule in a file called `/etc/udev/rules.d/99-pixhawk.rules` with the following content, changing the idVendor and idProduct to yours.
    
    ```sh
    SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", SYMLINK+="ttyArduino"
    SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0011", SYMLINK+="ttyPixhawk"
    ```
    
    Finally, after a **reboot** you can be sure to know which device is what and put `/dev/ttyPixhawk` instead of `/dev/ttyUSB0` in your scripts.
    
    > **Note** Be sure to add yourself in the `tty` and `dialout` groups via `usermod` to avoid to have to execute scripts as root.
    
    ```sh
    usermod -a -G tty ros-user
    usermod -a -G dialout ros-user
    ```