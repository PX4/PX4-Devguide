# UAVCAN 펌웨어 업그레이드

## Vectorcontrol ESC 코드 베이스 (픽스호크 ESC 1.6과 S2740VC)

ESC 코드를 다운로드하십시오:

```sh
git clone https://github.com/thiemar/vectorcontrol
cd vectorcontrol
```

### UAVCAN 부트로더 플래싱

UAVCAN으로 펌웨어 업그레이드를 진행하기 전, 픽스호크 ESC 1.6에 UAVCAN 부트로더를 플래싱해야합니다. 부트로더를 빌드하려면 다음 명령을 실행하십시오:

```sh
make clean && BOARD=px4esc_1_6 make -j8
```

빌드가 끝나면 부트로더 이미지는 `firmware/px4esc_1_6-bootloader.bin`에 들어가고, OpenOCD 설정은 `openocd_px4esc_1_6.cfg`에 들어갑니다. ESC 부트로더를 설치하려면 [이 절차](../uavcan/bootloader_installation.md)를 따르십시오.

### 메인 바이너리 컴파일

```sh
BOARD=s2740vc_1_0 make && BOARD=px4esc_1_6 make
```

이 명령은 지원하는 ESC용 UAVCAN 노드 펌웨어를 빌드합니다. 펌웨어 이미지는 `com.thiemar.s2740vc-v1-1.0-1.0.<git hash>.bin`과 `org.pixhawk.px4esc-v1-1.6-1.0.<git hash>.bin`으로 들어갑니다.

## Sapog 코드 베이스 (픽스호크 ESC 1.4와 Zubax Orel 20)

Sapog 코드 베이스를 다운로드하십시오:

```sh
git clone https://github.com/PX4/sapog
cd sapog
git submodule update --init --recursive
```

### UAVCAN 부트로더 플래싱

UAVCAN으로 펌웨어를 업데이트하기 전, ESC에 UAVCAN 부트로더를 플래싱해야합니다. 다음 명령어로 부트로더를 빌드할 수 있습니다:

```sh
cd bootloader
make clean && make -j8
cd ..
```

The bootloader image is located at `bootloader/firmware/bootloader.bin`, and the OpenOCD configuration is located at `openocd.cfg`. Follow [these instructions](../uavcan/bootloader_installation.md) to install the bootloader on the ESC.

### Compiling the Main Binary

```sh
cd firmware
make RELEASE=1 # RELEASE is optional; omit to build the debug version
```

Beware, some newer version of GCC lead to segfaults during linking. Version 4.9 did work at the time of writing. The firmware image will be located at `firmware/build/io.px4.sapog-1.1-1.7.<xxxxxxxx>.application.bin`, where `<xxxxxxxx>` is an arbitrary sequence of numbers and letters. There are two hardware version of the Zubax Orel 20 (1.0 and 1.1). Make sure you copy the binary to the correct folder in the subsequent description. The ESC firmware will check the hardware version and works on both products.1

## Zubax GNSS

Please refer to the [project page](https://github.com/Zubax/zubax_gnss) to learn how to build and flash the firmware. Zubax GNSS comes with a UAVCAN-capable bootloader, so its firmware can be updated in a uniform fashion via UAVCAN as described below.

## Firmware Installation on the Autopilot

The UAVCAN node file names follow a naming convention which allows the Pixhawk to update all UAVCAN devices on the network, regardless of manufacturer. The firmware files generated in the steps above must therefore be copied to the correct locations on an SD card or the PX4 ROMFS in order for the devices to be updated.

The convention for firmware image names is:

    <uavcan name>-<hw version major>.<hw version minor>-<sw version major>.<sw version minor>.<version hash>.bin
    

e.g. `com.thiemar.s2740vc-v1-1.0-1.0.68e34de6.bin`

However, due to space/performance constraints (names may not exceed 28 charates), the UAVCAN firmware updater requires those filenames to be split and stored in a directory structure like the following:

    /fs/microsd/fw/<node name>/<hw version major>.<hw version minor>/<hw name>-<sw version major>.<sw version minor>.<git hash>.bin
    

e.g.

    s2740vc-v1-1.0.68e34de6.bin 
    /fs/microsd/fw/io.px4.sapog/1.1/sapog-1.7.87c7bc0.bin
    

The ROMFS-based updater follows that pattern, but prepends the file name with ```_``` so you add the firmware in:

    /etc/uavcan/fw/<device name>/<hw version major>.<hw version minor>/_<hw name>-<sw version major>.<sw version minor>.<git hash>.bin
    

## Placing the binaries in the PX4 ROMFS

The resulting finale file locations are:

* S2740VC ESC: `ROMFS/px4fmu_common/uavcan/fw/com.thiemar.s2740vc-v1/1.0/_s2740vc-v1-1.0.<git hash>.bin`
* Pixhawk ESC 1.6: `ROMFS/px4fmu_common/uavcan/fw/org.pixhawk.px4esc-v1/1.6/_px4esc-v1-1.6.<git hash>.bin`
* Pixhawk ESC 1.4: `ROMFS/px4fmu_common/uavcan/fw/org.pixhawk.sapog-v1/1.4/_sapog-v1-1.4.<git hash>.bin``
* Zubax GNSS v1: `ROMFS/px4fmu_common/uavcan/fw/com.zubax.gnss/1.0/gnss-1.0.<git has>.bin`
* Zubax GNSS v2: `ROMFS/px4fmu_common/uavcan/fw/com.zubax.gnss/2.0/gnss-2.0.<git has>.bin`

Note that the ROMFS/px4fmu_common directory will be mounted to /etc on Pixhawk.

### Starting the Firmware Upgrade process

When using the PX4 Flight Stack, enable UAVCAN in the 'Power Config' section and reboot the system before attempting an UAVCAN firmware upgrade.

Alternatively UAVCAN firmware upgrading can be started manually on NSH via:

```sh
uavcan start
uavcan start fw
```