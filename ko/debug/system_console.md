# PX4 시스템 콘솔

PX4 *시스템 콘솔*에서는 시스템 저수준 접근이 가능하며, 디버깅 출력과 시스템 부팅 과정 분석을 진행할 수 있습니다.

> **Tip** 시스템이 부팅하지 않는지 디버깅하려면 콘솔을 사용해야합니다. [MAVLink 셸](../debug/mavlink_shell.md)은 아마도 설치하기 쉽고 [동일한 여러 작업 처리](../debug/consoles.md#console_vs_shell)에 활용할 수 있기에 다용도로 적격이라 봅니다.

## 콘솔 연결

The console is made available through a (board-specific) UART that can be connected to a computer USB port using a [3.3V FTDI](https://www.digikey.com/product-detail/en/TTL-232R-3V3/768-1015-ND/1836393) cable. This allows the console to be accessed using a terminal application.

Pixhawk controller manufacturers are expected to expose the console UART and SWD (JTAG) debug interfaces through a dedicated *debug port* that complies with the [Pixhawk Connector Standard](#pixhawk_debug_port). Unfortunately some boards predate this standard or a non-compliant.

> **Tip** Developers targeting a number of different boards may wish to use a *debug adapter* to simplify connecting multiple boards. For example, the [Dronecode probe](https://kb.zubax.com/display/MAINKB/Dronecode+Probe+documentation) comes with connectors for the [Pixhawk Debug Port](#pixhawk_debug_port) and several other boards.

The sections below outline/link to the wiring and system console information for many common boards.

### Board-Specific Wiring

The System Console UART pinouts/debug ports are typically documented in [autopilot overview pages](https://docs.px4.io/master/en/flight_controller/) (some are linked below):

- [3DR Pixhawk v1 Flight Controller](https://docs.px4.io/master/en/flight_controller/pixhawk.html#console-port) (also applies to [mRo Pixhawk](https://docs.px4.io/master/en/flight_controller/mro_pixhawk.html#debug-ports), [HobbyKing HKPilot32](https://docs.px4.io/master/en/flight_controller/HKPilot32.html#debug-port))
- [Pixhawk 3](https://docs.px4.io/master/en/flight_controller/pixhawk3_pro.html#debug-port)
- [Pixracer](https://docs.px4.io/master/en/flight_controller/pixracer.html#debug-port)

- [스냅드래곤 플라이트](https://docs.px4.io/master/en/flight_controller/snapdragon_flight.html):
  
  - [FTDI](https://docs.px4.io/master/en/flight_controller/snapdragon_flight_advanced.html#over-ftdi)
  - [DSP 디버깅 모니터/콘솔](https://docs.px4.io/master/en/flight_controller/snapdragon_flight_advanced.html#dsp-debug-monitorconsole)

### 픽스호크 디버깅 포트 {#pixhawk_debug_port}

Flight controllers that adhere to the Pixhawk Connector standard use the [Pixhawk Standard Debug Port](https://pixhawk.org/pixhawk-connector-standard/#dronecode_debug).

The port/FTDI mapping is shown below.

| 픽스호크 디버깅 포트 | -                        | FTDI | -                                 |
| ----------- | ------------------------ | ---- | --------------------------------- |
| 1 (적)       | TARGET PROCESSOR VOLTAGE |      | N/C (used for SWD/JTAG debugging) |
| 2 (blk)     | CONSOLE TX (OUT)         | 5    | FTDI RX (yellow)                  |
| 3 (blk)     | CONSOLE RX (IN)          | 4    | FTDI TX (orange)                  |
| 4 (blk)     | SWDIO                    |      | N/C (used for SWD/JTAG debugging) |
| 5 (blk)     | SWCLK                    |      | N/C (used for SWD/JTAG debugging) |
| 6 (blk)     | GND                      | 1    | FTDI GND (black)                  |

## Opening the Console

After the console connection is wired up, use the default serial port tool of your choice or the defaults described below:

### Linux / Mac OS: Screen

Install screen on Ubuntu (Mac OS already has it installed):

```bash
sudo apt-get install screen
```

- Serial: Pixhawk v1 / Pixracer use 57600 baud
- Serial: Snapdragon Flight uses 115200 baud

Connect screen at BAUDRATE baud, 8 data bits, 1 stop bit to the right serial port (use `ls /dev/tty*` and watch what changes when unplugging / replugging the USB device). Common names are `/dev/ttyUSB0` and `/dev/ttyACM0` for Linux and `/dev/tty.usbserial-ABCBD` for Mac OS.

```bash
screen /dev/ttyXXX BAUDRATE 8N1
```

### Windows: PuTTY

Download [PuTTY](http://www.chiark.greenend.org.uk/~sgtatham/putty/download.html) and start it.

Then select 'serial connection' and set the port parameters to:

- 57600 baud
- 8 data bits
- 1 stop bit