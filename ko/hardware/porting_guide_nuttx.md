# NuttX 보드 이식 안내

새 대상 하드웨어에 NuttX용 PX4를 이식하려면, 해당 대상 하드웨어가 NuttX에서 지원해야합니다. NuttX 프로젝트는 NuttX를 새 처리 플랫폼에 이식하는 최상의 [이식 안내서](https://cwiki.apache.org/confluence/display/NUTTX/Porting+Guide)를 관리하고 있습니다.

다음 안내서에서는 이미 지원 하드웨어 대상을 선정했고, 해당 하드웨어에 NuttX(와 [PX4 기반 계층](https://github.com/PX4/Firmware/tree/master/platforms/nuttx/src/px4))를 이식 했음을 가정합니다.

모든 보드를 대상으로 한 설정 파일, 링커 스크립트와 기타 필요한 설정은 제조사별 보드별 디렉터리(i.e. **boards/_VENDOR_/_MODEL_/**)의 [/boards](https://github.com/PX4/Firmware/tree/master/boards/)에 있습니다.

다음 예제는 NuttX 기반 비행체 조종 장치 용도로 최신 [기준 참고 설정](../hardware/reference_design.md)인 FMUv5를 활용합니다:
* **Firmware** 디렉터리에서 `make px4_fmu-v5_default` 명령을 실행하면 FMUv5 설정을 빌드합니다
* The base FMUv5 configuration files are located in: [/boards/px4/fmu-v5](https://github.com/PX4/Firmware/tree/master/boards/px4/fmu-v5).
  * Board specific header (NuttX pins and clock configuration): [/boards/px4/fmu-v5/nuttx-config/include/board.h](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v5/nuttx-config/include/board.h).
  * Board specific header (PX4 configuration): [/boards/px4/fmu-v5/src/board_config.h](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v5/src/board_config.h).
  * NuttX OS config (created with NuttX menuconfig): [/boards/px4/fmu-v5/nuttx-config/nsh/defconfig](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v5/nuttx-config/nsh/defconfig).
  * Build configuration: [boards/px4/fmu-v5/default.cmake](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v5/default.cmake).

## NuttX Menuconfig Setup

To modify the NuttX OS configuration, you can use [menuconfig](https://bitbucket.org/nuttx/nuttx) using the PX4 shortcuts:
```sh
make px4_fmu-v5_default menuconfig
make px4_fmu-v5_default qconfig
```

For fresh installs of PX4 onto Ubuntu using [ubuntu.sh](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/Tools/setup/ubuntu.sh) you will also need to install *kconfig* tools from [NuttX tools](https://bitbucket.org/nuttx/tools/src/master/).

> **Note** The following steps are not required if using the [px4-dev-nuttx](https://hub.docker.com/r/px4io/px4-dev-nuttx/) docker container or have installed to macOS using our normal instructions (as these include`kconfig-mconf`).

Run the following commands from any directory:
```sh
git clone https://bitbucket.org/nuttx/tools.git
cd tools/kconfig-frontends
sudo apt install gperf
./configure --enable-mconf --disable-nconf --disable-gconf --enable-qconf --prefix=/usr
make
sudo make install
```

The `--prefix=/usr` determines the specific installation location (which must be in the `PATH` environment variable). The `--enable-mconf` and `--enable-qconf` options will enable the `menuconfig` and `qconfig` options respectively.

To run `qconfig` you may need to install additional Qt dependencies.

### Bootloader

First you will need a bootloader, which depends on the hardware target:
- STM32H7: the bootloader is based on NuttX, and is included in the PX4 Firmware. See [here](https://github.com/PX4/Firmware/tree/master/boards/holybro/durandal-v1/nuttx-config/bootloader) for an example.
- For all other targets, https://github.com/PX4/Bootloader is used. See [here](https://github.com/PX4/Bootloader/pull/155/files) for an example how to add a new target. Then checkout the [buiding and flashing instructions](../software_update/stm32_bootloader.md).

### Firmware Porting Steps

1. Make sure you have a working [development setup](../setup/dev_env.md) and installed the NuttX menuconfig tool (see above).
1. Download the source code and make sure you can build an existing target:
   ```bash
   git clone --recursive https://github.com/PX4/Firmware.git
   cd Firmware
   make px4_fmu-v5
   ```
1. Find an existing target that uses the same (or a closely related) CPU type and copy it. For example for STM32F7:
   ```bash
   mkdir boards/manufacturer
   cp -r boards/px4/fmu-v5 boards/manufacturer/my-target-v1
   ```
   Change **manufacturer** to the manufacturer name and **my-target-v1** to your board name.

Next you need to go through all files under **boards/manufacturer/my-target-v1** and update them according to your board.
1. **firmware.prototype**: update the board ID and name
1. **default.cmake**: update the **VENDOR** and **MODEL** to match the directory names (**my-target-v1**). Configure the serial ports.
1. Configure NuttX (**defconfig**) via `make manufacturer_my-target-v1 menuconfig`: Adjust the CPU and chip, configure the peripherals (UART's, SPI, I2C, ADC).
1. **nuttx-config/include/board.h**: Configure the NuttX pins. For all peripherals with multiple pin options, NuttX needs to know the pin. They are defined in the chip-specific pinmap header file, for example [stm32f74xx75xx_pinmap.h](https://github.com/PX4/NuttX/blob/px4_firmware_nuttx-8.2/arch/arm/src/stm32f7/hardware/stm32f74xx75xx_pinmap.h).
1. **src**: go through all files under **src** and update them as needed, in particular **board_config.h**.
1. **init/rc.board_sensors**: start the sensors that are attached to the board.

