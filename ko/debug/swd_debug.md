# SWD (JTAG) 하드웨어 디버깅 인터페이스

PX4는 보통 ARM 칩의 *직렬 회선 디버깅(SWD)*을 지원하는 오토파일럿 조종 장치 하드웨어에서 실행합니다. SWD는 ARM 프로세서에서 JTAG 디버깅을 수행할 때 적은 수의 핀을 물리 인터페이스로 연결하는 방식입니다. SWD 호환 디버깅 프로브로 PX4의 중단점을 설정하고 실제 장치에서 코드 실행을 단계별로 진행할 때 활용할 수 있습니다.

SWD 인터페이스는 새 부트로더 또는 펌웨어를 완전히 깡통인 (USB 부트로더를 설치한 적이 없는) 보드에 추가할 때 사용할 수 있습니다.

이 주제에서는 다양한 보드로의 SWD 인터페이스 연결 방법(실제 디버깅 수행 방법은 관련 [디버깅 주제](#debugging_topics)에서 다룸)을 설명합니다.

## SWD 인터페이스 정의  {#swd_interface}

SWD 인터페이스는 다음 핀으로 이루어져있습니다.

| 핀       | 신호 형식 | 설명                                                                                                                                             |
| ------- | ----- | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| `Vref`  | 출력    | 대상 참조 전압. <br>일부 JTAG 어댑터에는 SWD 라인에 전압을 설정할 때 `Vref` 전압이 필요합니다. 예를 들면,  [SEGGER J-Link Debug Probes](#segger_jlink_edu_mini) 에서 `Vref`가 필요합니다. |
| `SWDIO` | 입출력   | 단일 전이중 데이터 핀.                                                                                                                                  |
| `SWCLK` | 출력    | 클록 신호.                                                                                                                                         |
| `GND`   | -     | 접지핀.                                                                                                                                           |


SWD의 "일부"는 아니지만, 오토파일럿에서는 *직렬 회선 출력 (SWO)* 추적 출력 핀이 있습니다. 만약 이 핀이 있다면 이 핀도 연결해야합니다.

| 핀     | 신호 형식 | 설명                                                       |
| ----- | ----- | -------------------------------------------------------- |
| `SWO` | 출력    | 직렬 회선 출력 방식의 추적 출력 핀. 실시간 추적 데이터를 인가하려면 *SWD*도 연결해야 합니다. |


## Connecting SWD Debuggers to PX4 Hardware

Connect `SWDIO`, `SWCLK` and `GND` pins on the debug probe to the corresponding pins on the autopilot.

In addition:
- Connect the `VRef` pin, if required by the debug adapter that is being used.
- Connect the `SWO` pin, if present.

Some SWD [debug probes](#debug_probes) come with adapters/cables for connecting to common Pixhawk [debug ports](#debug_ports). You can also create custom cables for connecting to different boards or probes.

> **Note** Some manufacturers provide cables to make it easy to connect the SWD interface and [System Console](../debug/system_console.md). For example the [CUAV V5nano](http://docs.px4.io/master/en/flight_controller/cuav_v5_nano.html#debug_port) and [CUAV V5+](http://docs.px4.io/master/en/flight_controller/cuav_v5_plus.html#debug-port) include this debug cable: ![6-pin JST SH Cable](../../assets/debug/cuav_v5_debug_cable.jpg)

<span></span>

> **Tip** Where possible, we highly recommend that you create or obtain an adapter board rather than custom cables for connecting to SWD/JTAG debuggers and computers. This reduces the risk or poor wiring contributing to debugging problems, and has the benefit that adapters usually provide a common interface for connecting to multiple popular flight controller boards.


## Autopilot Debug Ports {#debug_ports}

Flight controllers commonly provide a debug port that exposes both the [SWD Interface](#swd_interface) and [System Console](../debug/system_console.md).

The [Pixhawk Connector Standards](#pixhawk_standard_debug_ports) attempts to standardise this port. However since many boards use different pinouts or connectors, we recommend you check the documentation for your autopilot to confirm port location and pinout.

The debug port location and pinouts for a small subset of autopilots are linked below:

<span id="port_information"></span>

| 오토파일럿                                                                                                  | 커넥터                                                                                                                                                                                                                       |
| ------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [3DR Pixhawk](http://docs.px4.io/master/en/flight_controller/pixhawk.html#swd-port)                    | ARM 10-pin JTAG Connector (*mRo Pixhawk*, *HobbyKing HKPilot32*등의 FMUv2 보드에서도 사용함).                                                                                                                                       |
| [CUAV V5nano](http://docs.px4.io/master/en/flight_controller/cuav_v5_nano.html#debug_port)             | 6-pin JST GH<br>Digikey: [BM06B-GHS-TBT(LF)(SN)(N)](https://www.digikey.com/products/en?keywords=455-1582-1-ND) (수직 마운트), [SM06B-GHS-TBT(LF)(SN)(N)](https://www.digikey.com/products/en?keywords=455-1568-1-ND) (측면 마운트) |
| [CUAV V5+](http://docs.px4.io/master/en/flight_controller/cuav_v5_plus.html#debug-port)                | 6-pin JST GH<br>Digikey: [BM06B-GHS-TBT(LF)(SN)(N)](https://www.digikey.com/products/en?keywords=455-1582-1-ND) (수직 마운트), [SM06B-GHS-TBT(LF)(SN)(N)](https://www.digikey.com/products/en?keywords=455-1568-1-ND) (측면 마운트) |
| [Drotek Pixhawk 3 Pro](http://docs.px4.io/master/en/flight_controller/pixhawk3_pro.html#debug-port)    | [Pixhawk 6-pin SH Debug](#pixhawk_debug_port_6_pin_sh)                                                                                                                                                                    |
| [Holybro Pixhawk 4](http://docs.px4.io/master/en/flight_controller/pixhawk4.html#debug_port)           | [Pixhawk 6-pin SH Debug](#pixhawk_debug_port_6_pin_sh)                                                                                                                                                                    |
| [Holybro Pixhawk 4 Mini](http://docs.px4.io/master/en/flight_controller/pixhawk4_mini.html#debug-port) | [Pixhawk 6-pin SH Debug](#pixhawk_debug_port_6_pin_sh)                                                                                                                                                                    |
| [Holybro Kakute F7](http://docs.px4.io/master/en/flight_controller/kakutef7.html#debug-port)           | Solder pads                                                                                                                                                                                                               |
| [Holybro Durandal](http://docs.px4.io/master/en/flight_controller/durandal.html#debug-port)            | [Pixhawk 6-pin SH Debug](#pixhawk_debug_port_6_pin_sh)                                                                                                                                                                    |

> **Tip** 사용 중인 비행 조종 장치가 없다면 [오토파일럿 주제](http://docs.px4.io/master/en/flight_controller/)를 확인하십시오.



## Pixhawk 표준 디버깅 포트 {#pixhawk_standard_debug_ports}

The Pixhawk project has defines a standard pinout and connector type for different Pixhawk FMU releases:

> **Tip** Check your [specific board](#port_information) to confirm the port used.

| FMU 버전 | 픽스호크 버전                                                                                     | 디버그 인터페이스                                        |
| ------ | ------------------------------------------------------------------------------------------- | ------------------------------------------------ |
| FMUv2  | [Pixhawk / Pixhawk 1](http://docs.px4.io/master/en/flight_controller/pixhawk.html#swd-port) | 10 pin ARM Debug                                 |
| FMUv3  | Pixhawk 2                                                                                   | 6 pin SUR Debug                                  |
| FMUv4  | Pixhawk 3                                                                                   | [6 pin SH Debug](#pixhawk_debug_port_6_pin_sh)   |
| FMUv5  | Pixhawk 4 FMUv5                                                                             | [6 pin SH Debug](#pixhawk_debug_port_6_pin_sh)   |
| FMUv5X | Pixhawk 5X                                                                                  | [10 pin SH Debug](#pixhawk_debug_port_10_pin_sh) |
| FMUv6  | Pixhawk 6                                                                                   | [10 pin SH Debug](#pixhawk_debug_port_10_pin_sh) |
| FMUv6X | Pixhawk 6                                                                                   | [10 pin SH Debug](#pixhawk_debug_port_10_pin_sh) |

> **Note** There FMU and Pixhawk versions are (only) consistent after FMUv5X.


### Pixhawk Debug Mini (6-Pin SH Debug Port) {#pixhawk_debug_port_6_pin_sh}

The [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf) defines a *6-Pin SH Debug Port* that provides access to both SWD pins and the [System Console](../debug/system_console.md).

> **Note** 이 디버깅 코드는 FMUv4와 FMUv5에서 활용합니다.

핀 출력은 아래와 같습니다 (SWD 핀은 강조함):

| 디버깅 포트 | 핀        |
| ------ | -------- |
| 1      | `Vtref`  |
| 2      | 콘솔 TX    |
| 3      | 콘솔 RX    |
| 4      | `SWDIO`  |
| 5      | `SWDCLK` |
| 6      | `GND`    |

디버깅 포트 정의에는 다음 결선 방법도 포함합니다(커넥터로 이어지는 보드):

| 디버깅 포트 | 핀                | 전압    |
| ------ | ---------------- | ----- |
| 패드     | 신호               | 볼트    |
| 1      | NRST (reset)     | +3.3V |
| 2      | GPIO1 (여분의 GPIO) | +3.3V |
| 3      | GPIO1 (여분의 GPIO) | +3.3V |

The socket is a *6-pin JST SH* - Digikey number: [BM06B-SRSS-TBT(LF)(SN)](https://www.digikey.com/products/en?keywords=455-2875-1-ND) (vertical mount), [SM06B-SRSS-TBT(LF)(SN)](https://www.digikey.com/products/en?keywords=455-1806-1-ND)(side mount).

You can connect to the debug port using a [cable like this one](https://www.digikey.com/products/en?keywords=A06SR06SR30K152A).

![6-pin JST SH Cable](../../assets/debug/cable_6pin_jst_sh.jpg)


### Pixhawk Debug Full (10-Pin SH Debug Port) {#pixhawk_debug_port_10_pin_sh}

The [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)  defines a *10-Pin SH Debug Port* that provides access to both SWD pins and the [System Console](../debug/system_console.md). This essentially moves the solder pads from beside the [Pixhawk 6-Pin SH Debug Port](#pixhawk_debug_port_6_pin_sh) into the connector, and also adds an SWO pin.

> **Note** This port is specified for use in FMUv5x, FMUv6, FMUv6x.

The pinout is as shown below (SWD pins highlighted):

| 디버깅 포트 | 핀        |
| ------ | -------- |
| 1      | `Vtref`  |
| 2      | 콘솔 TX    |
| 3      | 콘솔 RX    |
| 4      | `SWDIO`  |
| 5      | `SWDCLK` |
| 6      | *SWO*    |
| 7      | NFC GPIO |
| 8      | PH11     |
| 9      | nRST     |
| 10     | `GND`    |

The socket is a *10-pin JST SH* - Digikey number: [BM10B-SRSS-TB(LF)(SN)](https://www.digikey.com/products/en?keywords=455-1796-2-ND) (vertical mount) or [SM10B-SRSS-TB(LF)(SN)](https://www.digikey.com/products/en?keywords=455-1810-2-ND) (side mount).

You can connect to the debug port using a [cable like this one](https://www.digikey.com/products/en?keywords=A10SR10SR30K203A).
![10-pin JST SH Cable](../../assets/debug/cable_10pin_jst_sh.jpg) <!-- better to have image showing proper connections for SWD+SWO -->


## 디버깅 프루브 {#debug_probes}

다음 절에서는 PX4를 구동하는 오토파일럿에 연결할 일부 잘 알려진 디버깅 프루브 장비와 어댑터의 개요를 설명하겠습니다.


### Segger JLink EDU Mini Debug Probe {#segger_jlink_edu_mini}

[Segger JLink EDU Mini](https://www.segger.com/products/debug-probes/j-link/models/j-link-edu-mini/)는 값싸고 인기있는 SWD 디버깅 프루브입니다. 프루브 커넥터 핀 출력은 아래 그림과 같습니다([FTSH-105-01-F-DV-K](https://www.digikey.com/products/en?keywords=SAM8796-ND)같은 ARM 10-pin 미니 커넥터로 연결).

![connector_jlink_mini.png](../../assets/debug/connector_jlink_mini.png)

J-Link Edu Mini를 [Pixhawk 6-Pin SH Debug Port](#pixhawk_debug_port_6_pin_sh)에 핀별로 결선해서 연결하는 방법은 아래에 나와있습니다(참고로 `-` 표시는 SWD에 필요하지 않음을 나타냅니다).

| 디버깅 포트     | J-Link Mini |
| ---------- | ----------- |
| 1 (Vtref)  | 1           |
| 2 (콘솔 TX)  | -           |
| 3 (콘솔 RX)  | -           |
| 4 (SWDIO)  | 2           |
| 5 (SWDCLK) | 4           |
| 6 (GND)    | 3 또는 5      |

> **Tip** From the table above you can infer the connections for autopilots that do not use the standard port.

<!-- Image of SWD cable and connector to debug port? --> 


### 드론코드 프루브 {#dronecode_probe}

The [Dronecode Probe](https://kb.zubax.com/display/MAINKB/Dronecode+Probe+documentation) is a generic JTAG/SWD + UART console adapter compatible with most ARM Cortex based designs, and in particular with Pixhawk series flight controllers (and other hardware that PX4 supports).

The probe's USB interface exposes two separate virtual serial port interfaces: one for connecting to the [System Console](../debug/system_console.md) (UART) and the other for an embedded GDB server (SWD interface).

The probe provides a DCD-M connector cable for attaching to the [Pixhawk 6-Pin SH Debug Port](#pixhawk_debug_port_6_pin_sh).

> **Note** The *6-pos DF13* connector that comes with the probe cannot be used for SWD debugging (it is for using the System Console).

<span></span>
> **Note** The *Dronecode Probe* is based on the [Black Magic Probe](#black_magic_probe).


### 블랙매직 프루브 {#black_magic_probe}

The [Black Magic Probe](https://github.com/blacksphere/blackmagic/wiki) is much like the [Dronecode probe](#dronecode_probe) but does not come with the same adapters for directly connecting to Pixhawk series flight controllers.

Adapters can be purchased separately:
- [Drone Code Debug Adapter](https://1bitsquared.com/products/drone-code-debug-adapter) (1 BIT SQUARED).


## 다음 단계 {#debugging_topics}

이제 SWD 디버깅 프루브를 비행 조종 장치에 연결했습니다!

다음 주제에서는 온-타겟 디버깅 방법을 설명하도록 하겠습니다:

- [PX4용 MCU Eclipse/J-Link 디버깅](../debug/eclipse_jlink.md)

