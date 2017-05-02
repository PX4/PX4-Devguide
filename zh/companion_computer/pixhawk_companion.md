---
translated_page: https://github.com/PX4/Devguide/blob/master/en/companion_computer/pixhawk_companion.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
translated: true
---

# Pixhawk系列飞控板的协同计算机

官网英文原文地址： http://dev.px4.io/pixhawk-companion-computer.html

无论何种协同计算机（Raspberry Pi, Odroid, Tegra K1），与Pixhawk系列飞控板之间的接口是相同的：它们通过串口连接到Pixhawk上的`TELEM2`，这个端口专用于与协同计算机相连。连接的消息格式是[MAVLink](http://mavlink.org)。

## Pixhawk设置

参考下表，设置`SYS_COMPANION`参数（System参数组）

> **须知：** 变更参数后需要重启飞控使其生效。


- `0`：禁用TELEM2上的MAVLink输出（默认）
- `921600`：使能MAVLink输出，波特率：921600, 8N1（推荐）
- `157600`：使能MAVLink输出，OSD模式，波特率：57600
- `257600`：使能MAVLink输出，监听模式，波特率：57600

## 协同计算机设置

为了能够接收MAVLink消息，协同计算机需要运行一些和串口通讯的软件，最常用的是：

- [MAVROS](../ros/mavros_installation.md)：ROS
- [C/C++ example code](https://github.com/mavlink/c_uart_interface_example)：自定义的代码
- [MAVProxy](http://mavproxy.org)：在串口和UDP之间传输MAVLink

## 硬件设置

根据下面的说明连接串口。所有Pixhawk串口工作在3.3V，兼容5V。

> ** 警告： ** 许多现代协同计算机在UART端口仅支持1.8V的电压，并且可能在3.3V下损坏。使用电压转换器。大多数时候，可以使用的硬件串口有特定的功能（modem or console），在使用之前，需要在Linux下重新配置它们。


安全的做法是使用FTDI（USB转串口适配器），并按照下面说明连接它。这大多数时候都管用并且很容易设置。

| TELEM2 |         | FTDI    |        |
--- | --- | ---
|1         | +5V (red)|         | DO NOT CONNECT!   |
|2         | Tx  (out)| 5       | FTDI RX (yellow) (in)   |
|3         | Rx  (in) | 4       | FTDI TX (orange) (out)  |
|4         | CTS (in) |6       | FTDI RTS (green) (out) |
|5         | RTS (out)|2       | FTDI CTS (brown) (in) |
|6         | GND     | 1       | FTDI GND (black)   |

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

> If you do not find your device, unplug it, execute `lsusb`, plug it, execute `lsusb` again and see the added device.

Therefore, we can create a new UDEV rule in a file called `/etc/udev/rules.d/99-pixhawk.rules` with the following content, changing the idVendor and idProduct to yours.

```sh
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", SYMLINK+="ttyArduino"
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0011", SYMLINK+="ttyPixhawk"
```

Finally, after a **reboot** you can be sure to know which device is what and put `/dev/ttyPixhawk` instead of `/dev/ttyUSB0` in your scripts.

> Be sure to add yourself in the `tty` and `dialout` groups via `usermod` to avoid to have to execute scripts as root.

```sh
usermod -a -G tty ros-user
usermod -a -G dialout ros-user
```
