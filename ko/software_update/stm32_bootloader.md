# STM32 부트로더

PX4 부트로더 코드는 깃허브 [Bootloader](https://github.com/px4/bootloader)에 있습니다.

## 지원 보드

* FMUv2 (Pixhawk 1, STM32F4)
* FMUv3 (Pixhawk 2, STM32F4)
* FMUv4 (Pixracer 3 and Pixhawk 3 Pro, STM32F4)
* FMUv5 (Pixhawk 4, STM32F7)
* TAPv1 (TBA, STM32F4)
* ASCv1 (TBA, STM32F4)

## 부트로드 빌드하기

```bash
git clone https://github.com/PX4/Bootloader.git
cd Bootloader
git submodule init
git submodule update
make
```

위 과정을 거치면 Bootloader 디렉토리내에 지원되는 모든 보드에 대한 elf파일이 생성됩니다.

## 부트로더 플래싱하기

> **Warning** 몇몇 보드의 경우 JTAG / SWD 접속을 위해서는 올바른 파워 시퀀스가 매우 중요합니다. 아래 단계를 설명한 것과 같이 정확하게 수행하십시오.

다음 안내는 Blackmagic / Dronecode 프로브를 사용하는 경우에 유효합니다. 다른 JTAG 프로브는 유사하지만 다른 절차가 필요합니다. 부트로더를 플래싱하는 개발자는 반드시 관련된 지식을 숙지하고 있어야 합니다. 이를 어떻게 하는지를 모르는 경우에는 부트로더와 관련된 무언가를 변경하는 것이 정말로 필요한지 다시 한번 고려해보시기 바랍니다.

절차는 다음과 같습니다.

1. JTAG 케이블 연결 제거
2. USB 전원 케이블 연결
3. JTAG 케이블 연결

### Black Magic / Dronecode 프로브

#### Using the right serial port

* On LINUX: `/dev/serial/by-id/usb-Black_Sphere_XXX-if00`
* On MAC OS: Make sure to use the cu.xxx port, not the tty.xxx port: `tar ext /dev/tty.usbmodemDDEasdf`

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

These instructions are for the [J-Link GDB server](https://www.segger.com/jlink-gdb-server.html).

#### Prerequisites

[Download the J-Link software](https://www.segger.com/downloads/jlink) from the Segger website and install it according to their instructions.

#### Run the JLink GDB server

The command below is used to run the server for flight controllers that use the STM32F427VI SoC:

```bash
JLinkGDBServer -select USB=0 -device STM32F427VI -if SWD-DP -speed 20000
```

The `--device`/SoC for common targets is:

* **FMUv2, FMUv3, FMUv4, aerofc-v1, mindpx-v2:** STM32F427VI
* **px4_fmu-v4pro:** STM32F469II
* **px4_fmu-v5:** STM32F765II
* **crazyflie:** STM32F405RG

#### Connect GDB

```bash
arm-none-eabi-gdb
  (gdb) tar ext :2331
  (gdb) load aerofcv1_bl.elf
```

### Troubleshooting

If any of the commands above are not found, you are either not using a Blackmagic probe or its software is outdated. Upgrade the on-probe software first.

If this error message occurs:

    Error erasing flash with vFlashErase packet
    

Disconnect the target (while leaving JTAG connected) and run

```bash
mon tpwr disable
swdp_scan
attach 1
load tapv1_bl.elf
```

This will disable target powering and attempt another flash cycle.