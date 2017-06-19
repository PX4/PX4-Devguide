# UAVCAN Bootloader 설치하기

> **Warning** UAVCAN 장치는 미리 bootloader를 설치하여 배송됩니다. UAVCAN 장치를 개발하는게 아니라면 이번 섹션을 따라할 필요는 없습니다.

## 개요

PX4 프로젝트는 STM32 장치에 대해서 표준 UAVCAN bootloader를 포함하고 있습니다.

bootloader는 flash의 처음 8-16 KB를 차지하며 전원이 들어오면 가장 먼저 실행됩니다. 일반적으로 bootloader는 하위레벨 장치를 초기화하고 자동으로 CAN bus의 baud rate를 결정하며 고유의 node ID를 얻기 위해서 UAVCAN dynamic node ID client처럼 동작합니다. 그리고 application이 부트되기 전까지 flight controller의 확인을 위해 대기하게 됩니다.

이 절차를 통해 사용자의 개입없이도 유효하지 않거나 손상된 appliation 펌웨어로부터 UAVCAN 장치가 복구가능하도록 하며 또한 자동으로 펌웨어 업데이트도 허용합니다.

## 전제 조건

UAVCAN bootloader를 설치하거나 업데이트시 필요한 것들 :

* SWD 혹은 JTAG 인터페이스(장치에 따라 다름), 예를 들면, [BlackMagic Probe](http://www.blacksphere.co.nz/main/blackmagic)나 [ST-Link v2](http://www.st.com/internet/evalboard/product/251168.jsp);
* SWD와 JTAG 인터페이스를 UAVCAN 장치의 디버깅 포트에 연결하는 어답터 케이블;
* [지원하는 ARM 툴체인](../setup/dev_env.md).

## 장치 준비

아래 지시대로 장치를 연결할 수 없는 경우라면 장치에 설치된 펌웨어가 이미 MCU의 debug pin을 비활성화 시켰을 가능성이 있습니다. 이를 복구하기 위해서는 인터페이스의 NRST나 nSRST pin을(표준 ARM 20-pin 커넥터에서 pin 15) MCU의 NRST pin에 연결하는 것입니다. 여러분이 가지고 있는 장치의 회로도나 PCB 레이아웃을 구해보거나 제조사에 연락해서 상세한 내용을 파악할 수 있습니다.

## 설치

여러분 장치에 맞는 bootloader를 컴파일하거나 이미지를 구한 후라면 bootloader는 반드시 장치의 flash 메모리의 시작부분에 복사해야만 합니다.

이를 수행하는 절차는 사용하는 SWD나 JTAG 인터페이스에 따라 달라집니다.

## BlackMagic Probe

BlackMagic Proble의 [최신 펌웨어](https://github.com/blacksphere/blackmagic/wiki/Hacking)를 확인합니다.

UAVAN 장치와 컴퓨터에 probe를 연결합니다.

probe의 장치 이름을 식별해야합니다. 일반적으로 `/dev/ttyACM<x>`나 `/dev/ttyUSB<x>`와 같은 형태입니다.

UAVCAN 장치에 전원을 넣고 실행 :

<div class="host-code"></div>

```sh
arm-none-eabi-gdb /path/to/your/bootloader/image.elf
```

`gdb` 프롬프트에서 실행 :

<div class="host-code"></div>

```gdb
target extended /dev/ttyACM0
monitor connect_srst enable
monitor swdp_scan
attach 1
set mem inaccessible-by-default off
load
run
```

`monitor swdp_scan`가 에러를 반환하면, 전선 연결이 제대로 되었는지 확인하고 최신버전의 펌웨어를 설치했는지 확인합니다.

## ST-Link v2

최소한 0.9.0버전의 [OpenOCD](http://openocd.org)를 사용합니다.

ST-Link를 UAVCAN 장치와 컴퓨터에 연결합니다.

UAVCAN 장치에 전원을 넣고 실행 :

<div class="host-code"></div>

```sh
openocd -f /path/to/your/openocd.cfg &
arm-none-eabi-gdb /path/to/your/bootloader/image.elf
```

`gdb` 프롬프트에서 실행 :

<div class="host-code"></div>

```gdb
target extended-remote localhost:3333
monitor reset halt
set mem inaccessible-by-default off
load
run
```
## Segger J-Link Debugger

JLink Debugger를 UAVCAN 장치와 컴퓨터에 연결합니다.

UAVCAN 장치에 전원을 넣고 실행 :

<div class="host-code"></div>

```JLinkGDBServer -select USB=0 -device STM32F446RE -if SWD-DP -speed 20000 -vd```

Open a second terminal, navigate to the directory that includes the px4esc_1_6-bootloader.elf for the esc and run:

<div class="host-code"></div>

```arm-none-eabi-gdb px4esc_1_6-bootloader.elf```

At the `gdb` prompt, run:

<div class="host-code"></div>

```tar ext :2331
load
```
## SEGGER JLink Debugger로 Flash 삭제하기

복구하는 방법으로 flash를 지워서 펌웨어가 기본 parameter를 사용할 수 있게 하는 방법이 유용합니다. SEGGER 설치 디렉토리로 가서 JLinkExe를 띄우고 다음을 실행 :

    device <name-of-device>
    erase

`<name-of-device>`를 마이크로컨트롤러의 이름으로 대체합니다. 예를 들자면 Pixhawk ESC 1.6인 경우는 STM32F446RE, SV2470VC ESC인 경우는 STM32F302K8가 됩니다.
