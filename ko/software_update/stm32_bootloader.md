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

The instructions below are valid for a Blackmagic / Dronecode probe. Other JTAG probes will need different but similar steps. Developers attempting to flash the bootloader should have the required knowledge. If you do not know how to do this you probably should reconsider if you really need to change anything about the bootloader.

The sequence is

1. Disconnect the JTAG cable
2. Connect the USB power cable
3. Connect the JTAG cable

### Black Magic / Dronecode Probe

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