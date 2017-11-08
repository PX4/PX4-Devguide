# STM32 Bootloader

PX4 bootloader 코드는 Github [Bootloader](https://github.com/px4/bootloader) repository를 참조하세요.

## 지원 보드

  * FMUv1 (PX4FMU, STM32F4)
  * FMUv2 (Pixhawk 1, STM32F4)
  * FMUv3 (Pixhawk 2, STM32F4)
  * FMUv4 (Pixracer 3 and Pixhawk 3 Pro, STM32F4)
  * FMUv5 (Pixhawk 4, STM32F7)
  * TAPv1 (TBA, STM32F4)
  * ASCv1 (TBA, STM32F4)

## Bootloader 빌드하기

```bash
git clone https://github.com/PX4/Bootloader.git
cd Bootloader
make
```

이 과정이 지나면 모든 지원 보드에 대한 elf 파일들이 Bootloader 디렉토리에 나타납니다.

## Bootloader를 플래쉬

> IMPORTANT: JTAG / SWD 접근이 가능한 일부 보드에서는 정확한 전원 순서가 중요합니다. 아래 내용은 Blackmagic / Dronecode probe에 대해서 유효합니다. 다른 JTAG probe는 비슷하지만 다를 수 있습니다. bootloader를 플래쉬하는 개발자는 관련 지식이 필요합니다. 만약 어떻게 하는지 모른다면 bootloader를 변경하는 것이 실제로 필요한 것인지 다시 생각해보세요.

  * JTAG 케이블 연결 끊기
  * USB 파워 케이블 연결
  * JTAG 케이블 연결하기

### Black Magic / Dronecode Probe

#### 시리얼 포트 사용하기

  * LINUX: ```/dev/serial/by-id/usb-Black_Sphere_XXX-if00```
  * MAC OS: tty.xxx port가 아니라 cu.xxx port를 사용하는지 확인 : ```tar ext /dev/tty.usbmodemDDEasdf```

```bash
arm-none-eabi-gdb
  (gdb) tar ext /dev/serial/by-id/usb-Black_Sphere_XXX-if00
  (gdb) mon swdp_scan
  (gdb) attach 1
  (gdb) mon option erase
  (gdb) mon erase_mass
  (gdb) load tapv1_bl.elf
        ...
        Transfer rate: 17 KB/sec, 828 bytes/write.
  (gdb) kill
```

### J-Link

[J-Link GDB server](https://www.segger.com/jlink-gdb-server.html)에 대한 설명

#### 전제 조건

Segger 사이트에서 [J-Link 소프트웨어 다운로드](https://www.segger.com/downloads/jlink)하고 지시에 따라 설치하기

#### JLink GDB server 실행하기

FMUv1:
```bash
JLinkGDBServer -select USB=0 -device STM32F405RG -if SWD-DP -speed 20000
```

AeroFC:
```bash
JLinkGDBServer -select USB=0 -device STM32F429AI -if SWD-DP -speed 20000
```

#### GDB 연결하기

```bash
arm-none-eabi-gdb
  (gdb) tar ext :2331
  (gdb) load aerofcv1_bl.elf
```

### 문제해결

위에 명령을 찾을 수 없다면 여러분이 사용하는 것이 Blackmagic이 아니거나 소프트웨어가 예전 버전일 수 있습니다. 먼저 on-probe 소프트웨어를 업데이트하세요.

만약 에러 메시지 발생:
```Error erasing flash with vFlashErase packet```

타겟 연결을 끊고(JTAG 연결은 두고) 실행

```bash
mon tpwr disable
swdp_scan
attach 1
load tapv1_bl.elf
```
이렇게 하면 타겟 전원을 비활성화시키고 다른 플래쉬 사이클을 시도합니다.
