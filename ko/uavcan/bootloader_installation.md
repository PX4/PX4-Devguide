# UAVCAN 부트로더 설치

> **Warning** UAVCAN 장치는 보통 부트로더를 미리 넣은 상태로 나옵니다. UAVCAN 장치를 직접 개발할 의도가 아니라면 이 절의 절차는 따르지 마십시오.

## 개요

PX4 프로젝트에서는 STM32 장치용 표준 UAVCAN 부트로더를 다룹니다.

부트로더는 처음 8~16KB 플래이 메모리 영역을 차지하며, 처음 코드는 전원을 켰을 때 실행합니다. 보통 부트로더는 로우레벨 장치 초기화, CAN 버스 전송률 자동 결정, 고유 노드 ID 획득을 위한 UAVCAN 동적 노드 ID 클라이언트 동작, 프로그램 부팅 진행 전 비행체 컨트롤러 응답 확인 과정을 수행합니다.

이 과정에서는 사용자의 개입 없이 UAVCAN 장치를 잘못된, 프로그램이 손상된 펌웨어 상태로부터 복원할 수 있으며, 자동 펌웨어 업데이트를 수행합니다.

## 준비 요건

Installing or updating the UAVCAN bootloader requires:

* An SWD or JTAG interface (depending on device), for example the [BlackMagic Probe](https://github.com/blacksphere/blackmagic/wiki) or the [ST-Link v2](http://www.st.com/internet/evalboard/product/251168.jsp);
* An adapter cable to connect your SWD or JTAG interface to the UAVCAN device's debugging port;
* A [supported ARM toolchain](../setup/dev_env.md).

## Device Preparation

If you are unable to connect to your device using the instructions below, it's possible that firmware already on the device has disabled the MCU's debug pins. To recover from this, you will need to connect your interface's NRST or nSRST pin (pin 15 on the standard ARM 20-pin connector) to your MCU's NRST pin. Obtain your device schematics and PCB layout or contact the manufacturer for details.

## 설치

After compiling or obtaining a bootloader image for your device (refer to device documentation for details), the bootloader must be copied to the beginning of the device's flash memory.

The process for doing this depends on the SWD or JTAG interface used.

## BlackMagic Probe

Ensure your BlackMagic Probe [firmware is up to date](https://github.com/blacksphere/blackmagic/wiki/Hacking).

Connect the probe to your UAVCAN device, and connect the probe to your computer.

Identify the probe's device name. This will typically be `/dev/ttyACM<x>` or `/dev/ttyUSB<x>`.

Power up your UAVCAN device, and run:

```sh
arm-none-eabi-gdb /path/to/your/bootloader/image.elf
```

At the `gdb` prompt, run:

```gdb
target extended /dev/ttyACM0
monitor connect_srst enable
monitor swdp_scan
attach 1
set mem inaccessible-by-default off
load
run
```

If `monitor swdp_scan` returns an error, ensure your wiring is correct, and that you have an up-to-date version of the BlackMagic firmware.

## ST-Link v2

Ensure you have a recent version—at least 0.9.0—of [OpenOCD](http://openocd.org).

Connect the ST-Link to your UAVCAN device, and connect the ST-Link to your computer.

Power up your UAVCAN device, and run:

```sh
openocd -f /path/to/your/openocd.cfg &
arm-none-eabi-gdb /path/to/your/bootloader/image.elf
```

At the `gdb` prompt, run:

```gdb
target extended-remote localhost:3333
monitor reset halt
set mem inaccessible-by-default off
load
run
```

## Segger J-Link 디버거

Connect the JLink Debugger to your UAVCAN device, and connect the JLink Debugger to your computer.

Power up your UAVCAN device, and run:

    JLinkGDBServer -select USB=0 -device STM32F446RE -if SWD-DP -speed 20000 -vd
    

Open a second terminal, navigate to the directory that includes the px4esc_1_6-bootloader.elf for the esc and run:

    arm-none-eabi-gdb px4esc_1_6-bootloader.elf
    

At the `gdb` prompt, run:

    tar ext :2331
    load
    

## Erasing Flash with SEGGER JLink Debugger

As a recovery method it may be useful to erase flash to factory defaults such that the firmware is using the default parameters. Go to the directory of your SEGGER installation and launch JLinkExe, then run:

    device <name-of-device>
    erase
    

Replace `<name-of-device>` with the name of the microcontroller, e.g. STM32F446RE for the Pixhawk ESC 1.6 or STM32F302K8 for the SV2470VC ESC.