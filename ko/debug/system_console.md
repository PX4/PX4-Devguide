# PX4 시스템 콘솔

PX4 *시스템 콘솔*에서는 시스템 저수준 접근이 가능하며, 디버깅 출력과 시스템 부팅 과정 분석을 진행할 수 있습니다.

> **Tip** 시스템이 부팅하지 않는지 디버깅하려면 콘솔을 사용해야합니다. [MAVLink 셸](../debug/mavlink_shell.md)은 아마도 설치하기 쉽고 [동일한 여러 작업 처리](../debug/consoles.md#console_vs_shell)에 활용할 수 있기에 다용도로 적격이라 봅니다.

## 콘솔 연결

콘솔은 [3.3V FTDI](https://www.digikey.com/product-detail/en/TTL-232R-3V3/768-1015-ND/1836393) 케이블을 활용하여 컴퓨터에 USB 포트를 연결할 수 있게 하는 (보드별) UART 포트로 띄울 수 있습니다. 이 포트는 터미널 프로그램으로 콘솔에 접근할 수 있게 해줍니다.

Pixhawk controller manufacturers are expected to expose the console UART and SWD (JTAG) debug interfaces through a dedicated *debug port* that complies with the [Pixhawk Connector Standard](#pixhawk_debug_port). Unfortunately some boards predate this standard or a non-compliant.

> **Tip** Developers targeting a number of different boards may wish to use a *debug adapter* to simplify connecting multiple boards. For example, the [Dronecode probe](https://kb.zubax.com/display/MAINKB/Dronecode+Probe+documentation) comes with connectors for the [Pixhawk Debug Port](#pixhawk_debug_port) and several other boards.

The sections below outline/link to the wiring and system console information for many common boards.

### 보드별 연결 방법

시스템 콘솔 UART 핀 출력/디버깅 포트는 [오토파일럿 개요 페이지](https://docs.px4.io/master/en/flight_controller/)에 정리해두었습니다(아래 링크에 있음):

- [3DR 픽스호크 v1 비행 조종 장치](https://docs.px4.io/master/en/flight_controller/pixhawk.html#console-port) ([mRo Pixhawk](https://docs.px4.io/master/en/flight_controller/mro_pixhawk.html#debug-ports), [HobbyKing HKPilot32](https://docs.px4.io/master/en/flight_controller/HKPilot32.html#debug-port)에도 적용)
- [픽스호크 3](https://docs.px4.io/master/en/flight_controller/pixhawk3_pro.html#debug-port)
- [픽스레이서](https://docs.px4.io/master/en/flight_controller/pixracer.html#debug-port)

- [스냅드래곤 플라이트](https://docs.px4.io/master/en/flight_controller/snapdragon_flight.html):
  
  - [FTDI](https://docs.px4.io/master/en/flight_controller/snapdragon_flight_advanced.html#over-ftdi)
  - [DSP 디버깅 모니터/콘솔](https://docs.px4.io/master/en/flight_controller/snapdragon_flight_advanced.html#dsp-debug-monitorconsole)

### 픽스호크 디버깅 포트 {#pixhawk_debug_port}

Flight controllers that adhere to the Pixhawk Connector standard use the [Pixhawk Standard Debug Port](https://pixhawk.org/pixhawk-connector-standard/#dronecode_debug).

The port/FTDI mapping is shown below.

| 픽스호크 디버깅 포트 | -                        | FTDI | -                        |
| ----------- | ------------------------ | ---- | ------------------------ |
| 1 (적)       | TARGET PROCESSOR VOLTAGE |      | 연결 안함 (SWD/JTAG 디버깅에 활용) |
| 2 (흑)       | CONSOLE TX (출력)          | 5    | FTDI RX (황)              |
| 3 (흑)       | CONSOLE RX (입력)          | 4    | FTDI TX (주황)             |
| 4 (흑)       | SWDIO                    |      | 연결 안함 (SWD/JTAG 디버깅에 활용) |
| 5 (흑)       | SWCLK                    |      | 연결 안함 (SWD/JTAG 디버깅에 활용) |
| 6 (흑)       | GND                      | 1    | FTDI GND (흑)             |

## 콘솔 열기

콘솔 연결을 끝내고 나면, 기본 직렬 포트 통신 도구를 취향에 따라 골라 사용하거나 아래 설명하는 기본 직렬 포트 통신 도구를 사용하십시오:

### Linux / Mac OS: Screen

우분투에 screen을 설치하십시오(Mac OS에는 이미 설치해둔 상태임):

```bash
sudo apt-get install screen
```

- 직렬 포트: 픽스호크 v1 / 픽스레이서 전송율: 57600 bps
- 직렬 포트: 스냅드래곤 플라이트의 경우 115200 bps

screen을 BAUDRATE bps, 데이터 비트 수 8, 정지 비트 1을 올바른 직렬 통신 포트에 설정하여 연결하여 연결하십시오(`ls /dev/tty*` 명령을 활용하여 USB 장치를 연결/분리 했을 때 어떤 값이 바뀌는지 확인). 일반 명칭은 리눅스의 경우 `/dev/ttyUSB0`과 `/dev/ttyACM0` 이며, Mac OS의 경우 `/dev/tty.usbserial-ABCBD`입니다.

```bash
screen /dev/ttyXXX BAUDRATE 8N1
```

### 윈도우: PuTTY

[PuTTY](http://www.chiark.greenend.org.uk/~sgtatham/putty/download.html)를 다운로드하고 시작하십시오.

이후 '직렬 연결'을 선택하고 포트 매개변수를 다음과 같이 설정하십시오:

- 초당 전송 비트: 57600
- 데이터 비트: 8
- 정지 비트: 1