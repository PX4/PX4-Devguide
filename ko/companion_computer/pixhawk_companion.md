# 픽스호크 시리즈용 보조 컴퓨터

픽스호크 계열 보드에 보조 컴퓨터(라즈베리 파이, 오드로이드, 테그라 K1)를 붙이는 작업의 방식은 동일합니다. 보조 컴퓨터를 연결하려는 용도의 포트 `TELEM 2`에 시리얼 포트로 연결하면 됩니다. 이 연결의 메시지 형식은 [MAVLink](https://mavlink.io/en/) 입니다.

## 픽스호크 설정

[설정 가능한 시리얼 포트](https://docs.px4.io/master/en/peripherals/serial_configuration.html)에 MAVLink를 활성화하십시오.

> **Tip** 보통 `TELEM 2` 포트는 보조 컴퓨터용으로 사용합니다.

`TELEM 2`의 기본 보조컴퓨터 메시지 스트림을 설정하려면 다음 매개변수를 설정하십시오:

* [MAV_1_CONFIG](../advanced/parameter_reference.md#MAV_1_CONFIG) = `TELEM 2` (`MAV_1_CONFIG`는 `TELEM 2` 포트 매핑 목적으로 주로 활용합니다)
* [MAV_1_MODE](../advanced/parameter_reference.md#MAV_1_MODE) = `Onboard`
* [SER_TEL2_BAUD](../advanced/parameter_reference.md#SER_TEL2_BAUD) = `921600` (로그 스트리밍 또는 FastRTPS 활용 목적으로 921600 이상 값을 권장합니다)

더 많은 정보는 [MAVLink 주변기기 편(GCS/OSD/Companion)](https://docs.px4.io/master/en/peripherals/mavlink_peripherals.html)을 참고하십시오.

## 보조 컴퓨터 설정

MAVLink 메시지를 받으려면, 보조 컴퓨터에서 직렬 포트로 통신하는 어떤 프로그램이 필요합니다. 대부분 일반적으로 선택할 수 있는 방안은 다음과 같습니다:

* ROS 노드와 통신 용도로 사용하는 [MAVROS](../ros/mavros_installation.md)
* 개별 작성 코드를 연결할 [C/C++ 예제 코드](https://github.com/mavlink/c_uart_interface_example)
* 시리얼 인터페이스와 UDP 통신간 라우팅 작업을 수행하는 [MAVLink 라우터](https://github.com/intel/mavlink-router)(추천) 또는 [MAVProxy](https://ardupilot.org/mavproxy/)

## 하드웨어 설정

다음 절차에 따라 직렬 포트를 연결하십시오. 모든 픽스호크 직렬 포트는 3.3V ~ 5V 레벨 호환으로 동작합니다.

> **Warning** 대부분의 최근 보조 컴퓨터는 1.8V 레벨에서 하드웨어 UART가 동작하며 3.3V 레벨에서 손상을 입을 수 있습니다. 전압 변환 칩을 활용하십시오. 대부분의 경우 접근할 수 있는 하드웨어 직렬 포트에는 연결하려는 장비(모뎀 또는 콘솔)와 관련된 일부 기능이 있으며, 해당 장비를 사용할 수 있으려면 *리눅스에서 재설정*해야합니다.

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