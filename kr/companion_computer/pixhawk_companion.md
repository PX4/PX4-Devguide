# Pixhawk 계열용 Companion Computer

companion computer(Raspberry Pi, Odroid, Tegra K1)를 Pixhawk 계열 보드에 연결은 항상 동일한 방식으로 동작합니다. 시리얼 포트 `TELEM2`를 사용해서 연결합니다. 이 링크에서 메시지 포맷은 [MAVLink](http://mavlink.org)입니다.

## Pixhawk 셋업

`SYS_COMPANION` 파라미터(System group 내에서)를 아래 값 중에 하나로 설정합니다.

> **Info** 이 파라미터를 변경하면 리부팅 후 설정이 반영된다.

  * `0` TELEM2로 MAVLink output을 비활성화 시킴(기본)
  * `921600` 921600 baud, 8N1로 MAVLink output 활성화 (추천)
  * `57600` 57600 baud, 8N1로 MAVLink output 활성화
  * `157600` *OSD* mode에서 57600 baud로 MAVLink 활성화
  * `257600` listen-only mode에서 57600 baud로 MAVLink 활성화

## Companion computer 셋업


MAVLink를 수신하기 위해서, companion computer는 시리얼 포트로 통신하는 소프트웨어를 실행해야 합니다. 가장 일반적인 옵션은 :

  * [MAVROS](../ros/mavros_installation.md) ROS node로 통신
  * [C/C++ example code](https://github.com/mavlink/c_uart_interface_example) 커스텀 코드로 연결
  * [MAVProxy](http://mavproxy.org) 시리얼과 UDP사이에 MAVLink를 라우팅

## Hardware 셋업

아래 지시에 따라서 시리얼 포트에 연결합니다. 모든 Pixhawk 시리얼 포트는 3.3V로 동작하며 5V까지 호환됩니다.

> ** Warning ** 최근 많은 companion computer의 하드웨어 UART는 1.8V만 지원해서 3.3V를 사용하는 경우 문제가 생길 수 있습니다. 이런 경우 레벨 쉬프터를 사용합니다. 일반적으로 하드웨어 시리얼 포트에는 이미 관련 기능(modem 이나 console)을 가지고 있어서 사용하기 전에 *Linux에서 재설정* 이 필요할 수 있다.

안정한 방법은 FTDI 칩의 USB-to-serial 아답터 보드를 사용하는 것입니다. 이 방식은 항상 동작하며 셋업하기 쉽습니다.

| TELEM2 |         | FTDI    |        |
--- | --- | ---
|1         | +5V (red)|         | DO NOT CONNECT!   |
|2         | Tx  (out)| 5       | FTDI RX (yellow) (in)   |
|3         | Rx  (in) | 4       | FTDI TX (orange) (out)  |
|4         | CTS (in) |6       | FTDI RTS (green) (out) |
|5         | RTS (out)|2       | FTDI CTS (brown) (in) |
|6         | GND     | 1       | FTDI GND (black)   |

## Linux에서 Software 셋업

Linux에서 USB FTDI의 기본 이름은 `\dev\ttyUSB0`와 같은 형태입니다. USB나 Arduino에 연결된 두번째 FTDI가 있는 경우라면, `\dev\ttyUSB1`로 등록될 수 있습니다. 첫번째와 두번째 연결 사이에 헷갈리는 것을 피하기 위해서 `ttyUSBx` 대신 USB 장치의 Vendor나 Product ID에 따라 친근한 이름의 symlink를 생성하는 것을 추천합니다.

`lsusb`를 사용하면 vendor와 product ID 정보를 얻을 수 있습니다.

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

Arduino는 `Bus 003 Device 004: ID 2341:0042 Arduino SA Mega 2560 R3 (CDC ACM)` 입니다.

Pixhawk는 `Bus 003 Device 005: ID 26ac:0011` 입니다.`

> 만약 장치가 보이지 않는다면, 플러그를 뽑고 `lsusb`를 실행하고, 다시 플러그를 연결하고 `lsusb`를 실행해서 추가된 장치를 확인합니다.

따라서 `/etc/udev/rules.d/99-pixhawk.rules` 파일에서 idVendor와 idProduct를 변경하기 위해서 다음 내용으로 새로운 UDEV rule을 생성할 수 있습니다.

```sh
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", SYMLINK+="ttyArduino"
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0011", SYMLINK+="ttyPixhawk"
```

마지막으로 **reboot** 후, 어떤 장치인지를 확인할 수 있어야 하고, script에 `/dev/ttyUSB0` 대신에 `/dev/ttyPixhawk`를 넣는다.

> `tty`와 `dialout` groups에 `usermod`로 여러분의 계정을 추가해야 root로 script가 실행되는 것을 피할 수 있습니다.

```sh
usermod -a -G tty ros-user
usermod -a -G dialout ros-user
```
