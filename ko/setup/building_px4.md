# PX4 소프트웨어 빌드

PX4는 모의시험 환경과 하드웨어 타겟 모두에 대해 콘솔 또는 IDE 환경에서 빌드할 수 있습니다.

> **Note** 다음 절차를 따르기 전에 우선 [개발자 툴체인](../setup/dev_env.md)을 호스트 운영 체제와 타겟 하드웨어용으로 설치해야합니다.

<span></span>

> **Tip** 일반 빌드 문제에 대한 해결책은 하단의 [문제 해결](#troubleshooting) 부준을 참고하십시오.

## PX4 소스 코드 다운로드 {#get_px4_code}

PX4 소스 코드는 github의 [PX4/Firmware](https://github.com/PX4/Firmware) 저장소에 있습니다. *가장 최신*의 버전을 컴퓨터에 받으려면, 다음 명령을 터미널에 입력하십시오:

```sh
git clone https://github.com/PX4/Firmware.git --recursive
```

> **Note** 이 방법이 최신 코드를 빌드하는데 필요한 모든 과정입니다. PX4에 기여할 목적의 더 많은 git 활용 내용은 [git 예제 > PX4에 코드 기여하기](../contribute/git_examples.md#contributing_code) 에 있습니다.

## 첫 빌드 (jMAVSim 모의시험 환경 활용) {#jmavsim_build}

우선 콘솔 환경에서 모의시험 환경을 빌드하겠습니다. 모의시험 환경은 실제 하드웨어와 IDE로 옮겨가기 전 시스템 설정을 검증할 수 있게 합니다.

**Firmware** 디렉터리를 찾아간 후 다음 명령으로 [jMAVSim](../simulation/jmavsim.md)을 시작하십시오:

```sh
make px4_sitl jmavsim
```

이 명령으로 다음의 PX4 콘솔을 띄웁니다:

![PX4 콘솔 (jMAVSim)](../../assets/console_jmavsim.png)

다음 명령을 입력하면 드론이 날 수 있습니다:

```sh
pxh> commander takeoff
```

![jMAVSim UI](../../assets/jmavsim_first_takeoff.png)

`commander land` 명령으로 드론을 착륙할 수 있으며 모의시험 환경은 **CTRL+C** 키 입력(또는 `shutdown` 명령 입력)으로 멈출 수 있습니다.

지상 관제 스테이션에서의 비행체 모의시험은 실제 비행체 운용과 거의 흡사합니다. 비행체가 날고 있을 때(비행체 이륙 모드) 지도에서 위치를 누르고 슬라이더를 활성화합니다. 이 동작을 통해 비행체의 위치를 바꿉니다.

![QGroundControl GoTo](../../assets/qgc_goto.jpg)

> **Tip** [가제보(Gazebo) 모의시험 환경](../simulation/gazebo.md), [AirSim 모의시험 환경](../simulation/airsim.md)과 같은 다른 여러 [모의시험 환경](../simulation/README.md)에서도 PX4를 활용할 수 있습니다. 이들 역시 *make* 명령으로 시작합니다. 예를 들면: ```make px4_sitl gazebo```

## NuttX / Pixhawk 기반 보드 {#nuttx}

### 빌드 {#building_nuttx}

NuttX- 또는 Pixhawk- 기반 보드용으로 빌드하려면 **Firmware** 디렉토리를 탐색한 후 보드에 해당하는 빌드 타겟을 찾아 `make`를 호출하십시오.

예를 들어 *Pixracer* 용으로 빌드하려면 다음 명령을 사용하십시오:

```sh
cd Firmware
make px4_fmu-v4_default
```

> **Note** 위 예제에서 빌드 타겟의 처음 부분인 `px4_fmu-v4`는 비행체 제어부 하드웨어 일부 기종용 펌웨어 이름이며, `default`는 설정 이름입니다 (이 경우 "default" 설정입니다). `default`는 선택 사항이기에, 대신 다음 명령을 실행할 수 있습니다: ```make px4_fmu-v4```

성공적인 실행시 다음 출력 내용으로 끝납니다:

```sh
-- Build files have been written to: /home/youruser/src/Firmware/build/px4_fmu-v4_default
[954/954] Creating /home/youruser/src/Firmware/build/px4_fmu-v4_default/px4_fmu-v4_default.px4
```

다음 목록은 일반 보드에서의 빌드 명령을 보여줍니다:

- [Pixhawk 4](http://docs.px4.io/master/en/flight_controller/pixhawk4.html): `make px4_fmu-v5_default`
- [Pixhawk 4 Mini](http://docs.px4.io/master/en/flight_controller/pixhawk4_mini.html): `make px4_fmu-v5_default`
- [CUAV V5+](http://docs.px4.io/master/en/flight_controller/cuav_v5_plus.html): `make px4_fmu-v5_default`
- [CUAV V5 nano](http://docs.px4.io/master/en/flight_controller/cuav_v5_nano.html): `make px4_fmu-v5_default`
- [Holybro Kakute F7](http://docs.px4.io/master/en/flight_controller/kakutef7.html): `make holybro_kakutef7_default`
- [Pixracer](https://docs.px4.io/master/en/flight_controller/pixracer.html): `make px4_fmu-v4_default`
- [Pixhawk 3 Pro](https://docs.px4.io/master/en/flight_controller/pixhawk3_pro.html): `make px4_fmu-v4pro_default`
- [Pixhawk Mini](https://docs.px4.io/master/en/flight_controller/pixhawk_mini.html): `make px4_fmu-v3_default`
- [Cube Black](https://docs.px4.io/master/en/flight_controller/pixhawk-2.html): `make px4_fmu-v3_default`
- Cube Yellow: `make hex_cube-yellow`
- Cube Orange: `make hex_cube-orange`
- [mRo Pixhawk](https://docs.px4.io/master/en/flight_controller/mro_pixhawk.html): `make px4_fmu-v3_default` (2MB 플래시 메모리 지원)
- [HKPilot32](https://docs.px4.io/master/en/flight_controller/HKPilot32.html): `make px4_fmu-v2_default`
- [Pixfalcon](https://docs.px4.io/master/en/flight_controller/pixfalcon.html): `make px4_fmu-v2_default`
- [Dropix](https://docs.px4.io/master/en/flight_controller/dropix.html): `make px4_fmu-v2_default`
- [MindPX](https://docs.px4.io/master/en/flight_controller/mindpx.html)/[MindRacer](https://docs.px4.io/master/en/flight_controller/mindracer.html): `make airmind_mindpx-v2_default`
- [mRo X-2.1](https://docs.px4.io/master/en/flight_controller/mro_x2.1.html): `make mro_x21_default` 
- [Crazyflie 2.0](https://docs.px4.io/master/en/flight_controller/crazyflie2.html): `make bitcraze_crazyflie_default`
- [Intel® Aero Ready to Fly Drone](https://docs.px4.io/master/en/flight_controller/intel_aero.html): `make intel_aerofc-v1_default`
- [Pixhawk 1](https://docs.px4.io/master/en/flight_controller/pixhawk.html): `make px4_fmu-v2_default` > **Warning** 이 보드를 대상으로 빌드하려면 지원하는 GCC 버전(예: [CI/docker](../test_and_ci/docker.md)에서 사용하는 버전과 동일)을 활용 **해야** 하거나, 빌드에서 모듈을 제거해야합니다. 지원하지 않는 버전의 GCC 로 빌드하면, PX4 보드의 1MB 플래시 용량 제한에 가까워져 실패할 수 있습니다.
- 2 MB flash의 Pixhawk 1: `make px4_fmu-v3_default`

> **Note** 보통 `_default` 접미사는 선택 입력사항입니다 (예: `make px4_fmu-v4`, `make bitcraze_crazyflie`, 등의 명령으로도 빌드할 수 있습니다.).

### 펌웨어 업로드 (보드 플래싱)

Append `upload` to the make commands to upload the compiled binary to the autopilot hardware via USB. For example

```sh
make px4_fmu-v4_default upload
```

성공적인 실행시 다음 출력 내용으로 끝납니다:

```sh
Erase  : [====================] 100.0%
Program: [====================] 100.0%
Verify : [====================] 100.0%
Rebooting.

[100%] Built target upload
```

## 기타 보드

다음 보드는 빌드 또는 배포 방법이 좀 더 복잡합니다.

### 라즈베리 파이 2/3 보드

아래 명령으로 [Raspberry Pi 2/3 Navio2](https://docs.px4.io/master/en/flight_controller/raspberry_pi_navio2.html) 대상 바이너리를 빌드합니다.

#### 교차 컴파일러 빌드

Set the IP (or hostname) of your RPi using:

```sh
export AUTOPILOT_HOST=192.168.X.X
```

또는

```sh
export AUTOPILOT_HOST=pi_hostname.domain
```

> **Note** The value of the environment variable should be set before the build, or `make upload` will fail to find your RPi.

Build the executable file:

```sh
cd Firmware
make emlid_navio2 # for cross-compiler build
```

The "px4" executable file is in the directory **build/emlid_navio2_default/**. Make sure you can connect to your RPi over ssh, see [instructions how to access your RPi](https://docs.px4.io/master/en/flight_controller/raspberry_pi_navio2.html#developer-quick-start).

Then upload it with:

```sh
cd Firmware
make emlid_navio2 upload # for cross-compiler build
```

Then, connect over ssh and run it with (as root):

```sh
sudo ./bin/px4 -s px4.config
```

#### 자체 빌드

If you're building *directly* on the Pi, you will want the native build target (emlid_navio2_native).

```sh
cd Firmware
make emlid_navio2_native # for native build
```

The "px4" executable file is in the directory **build/emlid_navio2_native/**. Run it directly with:

```sh
sudo ./build/emlid_navio2_native/px4 -s ./posix-configs/rpi/px4.config
```

A successful build followed by executing px4 will give you something like this:

```sh
<br />______  __   __    ___
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.


pxh>
```

#### 자동 시작

PX4를 자동으로 시작하려면 다음 실행 명령을 **/etc/rc.local**의 `exit 0` 행 바로 전에 추가하십시오(자체적으로 빌드했다면, 적당한 명령으로 수정하십시오):

```sh
cd /home/pi && ./bin/px4 -d -s px4.config > px4.log
```

### OcPoC-Zynq Mini

[OcPoC-Zynq Mini](https://docs.px4.io/master/en/flight_controller/ocpoc_zynq.html) 빌드 절차는 다음 문서에서 다룹니다:

- [Aerotenna OcPoC-Zynq Mini Flight Controller > OcPoC-Zynq용 PX4 빌드](https://docs.px4.io/master/en/flight_controller/ocpoc_zynq.html#building-px4-for-ocpoc-zynq)
- [OcPoC PX4 설정 페이지](https://aerotenna.readme.io/docs/px4-setup)

### QuRT / 스냅드래곤 기반 보드

이 절에서는 [Qualcomm Snapdragon Flight](https://docs.px4.io/master/en/flight_controller/snapdragon_flight.html)용 빌드 방법을 알려드립니다.

#### 빌드

> **Note** If you use the [Qualcomm ESC board](http://shop.intrinsyc.com/products/qualcomm-electronic-speed-control-board) (UART-based), then please follow their instructions [here](https://github.com/ATLFlight/ATLFlightDocs/blob/master/PX4.md). If you use normal PWM-based ESCs boards, then you may continue to follow the instructions on this page.

The commands below build the targets for the Linux and the DSP side. Both executables communicate via [muORB](../middleware/uorb.md).

```sh
cd Firmware
make atlflight_eagle_default
```

To load the SW on the device, connect via USB cable and make sure the device is booted. Run this in a new terminal window:

```sh
adb shell
```

Go back to previous terminal and upload:

```sh
make atlflight_eagle_default upload
```

Note that this will also copy (and overwrite) the two config files [mainapp.config](https://github.com/PX4/Firmware/blob/master/posix-configs/eagle/flight/mainapp.config) and [px4.config](https://github.com/PX4/Firmware/blob/master/posix-configs/eagle/flight/px4.config) to the device. Those files are stored under /usr/share/data/adsp/px4.config and /home/linaro/mainapp.config respectively if you want to edit the startup scripts directly on your vehicle.

The mixer currently needs to be copied manually:

```sh
adb push ROMFS/px4fmu_common/mixers/quad_x.main.mix  /usr/share/data/adsp
```

#### 실행

Run the DSP debug monitor:

```sh
${HEXAGON_SDK_ROOT}/tools/debug/mini-dm/Linux_Debug/mini-dm
```

Note: alternatively, especially on Mac, you can also use [nano-dm](https://github.com/kevinmehall/nano-dm).

Go back to ADB shell and run px4:

```sh
cd /home/linaro
./px4 -s mainapp.config
```

Note that the px4 will stop as soon as you disconnect the USB cable (or if you ssh session is disconnected). To fly, you should make the px4 auto-start after boot.

#### 자동 시작

To run the px4 as soon as the Snapdragon has booted, you can add the startup to `rc.local`:

Either edit the file `/etc/rc.local` directly on the Snapdragon:

```sh
adb shell
vim /etc/rc.local
```

Or copy the file to your computer, edit it locally, and copy it back:

```sh
adb pull /etc/rc.local
gedit rc.local
adb push rc.local /etc/rc.local
```

For the auto-start, add the following line before `exit 0`:

```sh
(cd /home/linaro && ./px4 -s mainapp.config > mainapp.log)

exit 0
```

Make sure that the `rc.local` is executable:

```sh
adb shell
chmod +x /etc/rc.local
```

Then reboot the Snapdragon:

```sh
adb reboot
```

## 그래픽 IDE에서의 컴파일

The PX4 system supports Qt Creator, Eclipse and Sublime Text. Qt Creator is the most user-friendly variant and hence the only officially supported IDE. Unless an expert in Eclipse or Sublime, their use is discouraged. Hardcore users can find an [Eclipse project](https://github.com/PX4/Firmware/blob/master/eclipse.project) and a [Sublime project](https://github.com/PX4/Firmware/blob/master/Firmware.sublime-project) in the source tree.

{% youtube %}https://www.youtube.com/watch?v=Bkk8zttWxEI&rel=0&vq=hd720{% endyoutube %}

## Qt Creator 기능 

Qt creator offers clickable symbols, auto-completion of the complete codebase and building and flashing firmware.

![](../../assets/toolchain/qtcreator.png)

### 리눅스용 Qt Creator

Before starting Qt Creator, the [project file](https://gitlab.kitware.com/cmake/community/wikis/doc/cmake/Generator-Specific-Information#codeblocks-generator) needs to be created:

```sh
cd ~/src/Firmware
mkdir ../Firmware-build
cd ../Firmware-build
cmake ../Firmware -G "CodeBlocks - Unix Makefiles"
```

Then load the CMakeLists.txt in the root firmware folder via **File > Open File or Project** (Select the CMakeLists.txt file).

After loading, the **play** button can be configured to run the project by selecting 'custom executable' in the run target configuration and entering 'make' as executable and 'upload' as argument.

### Windows용 Qt Creator

> **Note** Windows has not been tested for PX4 development with Qt Creator.

### Mac OS용 Qt Creator

Before starting Qt Creator, the [project file](https://gitlab.kitware.com/cmake/community/wikis/doc/cmake/Generator-Specific-Information#codeblocks-generator) needs to be created:

```sh
cd ~/src/Firmware
mkdir -p build/creator
cd build/creator
cmake ../.. -G "CodeBlocks - Unix Makefiles"
```

That's it! Start *Qt Creator*, then complete the steps in the video below to set up the project to build.

{% youtube %}https://www.youtube.com/watch?v=0pa0gS30zNw&rel=0&vq=hd720{% endyoutube %}

## PX4 make 빌드 타겟 {#make_targets}

앞 절에서는 *make*를 호출하여 제각기 다른 타겟을 빌드하고, 모의시험 환경을 시작하고 IDE를 활용하는 방법을 다루었습니다. 이 절에서는 *make* 옵션을 구성하는 방법과 존재하는 선택지를 찾는 방법을 다루도록 하겠습니다.

The full syntax to call *make* with a particular configuration and initialization file is:

```sh
make [VENDOR_][MODEL][_VARIANT] [VIEWER_MODEL_DEBUGGER_WORLD]
```

**VENDOR_MODEL_VARIANT**: (also known as `CONFIGURATION_TARGET`)

- **VENDOR:** The manufacturer of the board: `px4`, `aerotenna`, `airmind`, `atlflight`, `auav`, `beaglebone`, `intel`, `nxp`, etc. The vendor name for Pixhawk series boards is `px4`.
- **MODEL:** The *board model* "model": `sitl`, `fmu-v2`, `fmu-v3`, `fmu-v4`, `fmu-v5`, `navio2`, etc.
- **VARIANT:** Indicates particular configurations: e.g. `rtps`, `lpe`, which contain components that are not present in the `default` configuration. Most commonly this is `default`, and may be omitted.

> **Tip** You can get a list of *all* available `CONFIGURATION_TARGET` options using the command below: 
> 
>     sh
>       make list_config_targets

**VIEWER_MODEL_DEBUGGER_WORLD:**

- **VIEWER:** This is the simulator ("viewer") to launch and connect: `gazebo`, `jmavsim` <!-- , ?airsim -->

- **MODEL:** The *vehicle* model to use (e.g. `iris` (*default*), `rover`, `tailsitter`, etc), which will be loaded by the simulator. The environment variable `PX4_SIM_MODEL` will be set to the selected model, which is then used in the [startup script](..\simulation\README.md#scripts) to select appropriate parameters.

- **DEBUGGER:** Debugger to use: `none` (*default*), `ide`, `gdb`, `lldb`, `ddd`, `valgrind`, `callgrind`. For more information see [Simulation Debugging](../debug/simulation_debugging.md).
- **WORLD:** (Gazebo only). Set a the world ([PX4/sitl_gazebo/worlds](https://github.com/PX4/sitl_gazebo/tree/master/worlds)) that is loaded. Default is [empty.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/empty.world). For more information see [Gazebo > Loading a Specific World](../simulation/gazebo.md#set_world).

> **Tip** You can get a list of *all* available `VIEWER_MODEL_DEBUGGER_WORLD` options using the command below: 
> 
>     sh
>       make px4_sitl list_vmd_make_targets

Notes:

- Most of the values in the `CONFIGURATION_TARGET` and `VIEWER_MODEL_DEBUGGER` have defaults, and are hence optional. For example, `gazebo` is equivalent to `gazebo_iris` or `gazebo_iris_none`.
- You can use three underscores if you want to specify a default value between two other settings. For example, `gazebo___gdb` is equivalent to `gazebo_iris_gdb`.
- You can use a `none` value for `VIEWER_MODEL_DEBUGGER` to start PX4 and wait for a simulator. For example start PX4 using `make px4_sitl_default none` and jMAVSim using `./Tools/jmavsim_run.sh -l`.

The `VENDOR_MODEL_VARIANT` options map to particular *cmake* configuration files in the PX4 source tree under the [/boards](https://github.com/PX4/Firmware/tree/master/boards) directory. Specifically `VENDOR_MODEL_VARIANT` maps to a configuration file **boards/VENDOR/MODEL/VARIANT.cmake** (e.g. `px4_fmu-v5_default` corresponds to [boards/px4/fmu-v5/default.cmake](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v5/default.cmake)).

다음 절에서 추가 make 타겟을 다루도록 하겠습니다 (완전한 목록은 아님):

### 이진 파일 크기 프로파일링 {#bloaty_compare_master}

The `bloaty_compare_master` build target allows you to get a better understanding of the impact of changes on code size. When it is used, the toolchain downloads the latest successful master build of a particular firmware and compares it to the local build (using the [bloaty](https://github.com/google/bloaty) size profiler for binaries).

> **Tip** This can help analyse changes that (may) cause `px4_fmu-v2_default` to hit the 1MB flash limit.

*Bloaty* must be in your path and found at *cmake* configure time. The PX4 [docker files](https://github.com/PX4/containers/blob/master/docker/Dockerfile_nuttx-bionic) install *bloaty* as shown:

    git clone --recursive https://github.com/google/bloaty.git /tmp/bloaty \
        && cd /tmp/bloaty && cmake -GNinja . && ninja bloaty && cp bloaty /usr/local/bin/ \
        && rm -rf /tmp/*
    

The example below shows how you might see the impact of removing the *mpu9250* driver from `px4_fmu-v2_default`. First it locally sets up a build without the driver:

```sh
 % git diff
diff --git a/boards/px4/fmu-v2/default.cmake b/boards/px4/fmu-v2/default.cmake
index 40d7778..2ce7972 100644
--- a/boards/px4/fmu-v2/default.cmake
+++ b/boards/px4/fmu-v2/default.cmake
@@ -36,7 +36,7 @@ px4_add_board(
                imu/l3gd20
                imu/lsm303d
                imu/mpu6000

-               imu/mpu9250
+               #imu/mpu9250
                #iridiumsbd
                #irlock
                #magnetometer # all available magnetometer drivers
```

Then use the make target, specifying the target build to compare (`px4_fmu-v2_default` in this case):

```sh
% make px4_fmu-v2_default bloaty_compare_master
...
...
...
     VM SIZE                                                                                        FILE SIZE
 --------------                                                                                  --------------
  [DEL]     -52 MPU9250::check_null_data(unsigned int*, unsigned char)                               -52  [DEL]
  [DEL]     -52 MPU9250::test_error()                                                                -52  [DEL]
  [DEL]     -52 MPU9250_gyro::MPU9250_gyro(MPU9250*, char const*)                                    -52  [DEL]
  [DEL]     -56 mpu9250::info(MPU9250_BUS)                                                           -56  [DEL]
  [DEL]     -56 mpu9250::regdump(MPU9250_BUS)                                                        -56  [DEL]
...                                        -336  [DEL]
  [DEL]    -344 MPU9250_mag::_measure(ak8963_regs)                                                  -344  [DEL]
  [DEL]    -684 MPU9250::MPU9250(device::Device*, device::Device*, char const*, char const*, cha    -684  [DEL]
  [DEL]    -684 MPU9250::init()                                                                     -684  [DEL]
  [DEL]   -1000 MPU9250::measure()                                                                 -1000  [DEL]
 -41.3%   -1011 [43 Others]                                                                        -1011 -41.3%
  -1.0% -1.05Ki [Unmapped]                                                                       +24.2Ki  +0.2%
  -1.0% -10.3Ki TOTAL                                                                            +14.9Ki  +0.1%
```

This shows that removing *mpu9250* from `px4_fmu-v2_default` would save 10.3 kB of flash. It also shows the sizes of different pieces of the *mpu9250* driver.

## 펌웨어 버전과 git 태그 {#firmware_version}

The *PX4 Firmware Version* and *Custom Firmware Version* are published using the MAVLink [AUTOPILOT_VERSION](https://mavlink.io/en/messages/common.html#AUTOPILOT_VERSION) message, and displayed in the *QGroundControl* **Setup > Summary** airframe panel:

![펌웨어 정보](../../assets/gcs/qgc_setup_summary_airframe_firmware.jpg)

These are extracted at build time from the active *git tag* for your repo tree. The git tag should be formatted as `<PX4-version>-<vendor-version>` (e.g. the tag in the image above was set to `v1.8.1-2.22.1`).

> **Warning** If you use a different git tag format, versions information may not be displayed properly.

## 문제 해결 {#troubleshooting}

### 일반 빌드 오류

Many build problems are caused by either mismatching submodules or an incompletely cleaned-up build environment. Updating the submodules and doing a `distclean` can fix these kinds of errors:

    git submodule update --recursive
    make distclean
    

### Flash overflowed by XXX bytes

The `region 'flash' overflowed by XXXX bytes` error indicates that the firmware is too large for the target hardware platform. This is common for `make px4_fmu-v2_default` builds, where the flash size is limited to 1MB.

If you're building the *vanilla* master branch, the most likely cause is using an unsupported version of GCC. In this case, install the version specified in the [Developer Toolchain](../setup/dev_env.md) instructions.

If building your own branch, it is possibly you have increased the firmware size over the 1MB limit. In this case you will need to remove any drivers/modules that you don't need from the build.

### macOS: Too many open files error {#macos_open_files}

MacOS allows a default maximum of 256 open files in all running processes. The PX4 build system opens a large number of files, so you may exceed this number.

The build toolchain will then report `Too many open files` for many files, as shown below:

```sh
/usr/local/Cellar/gcc-arm-none-eabi/20171218/bin/../lib/gcc/arm-none-eabi/7.2.1/../../../../arm-none-eabi/bin/ld: cannot find NuttX/nuttx/fs/libfs.a: Too many open files
```

The solution is to increase the maximum allowed number of open files (e.g. to 300). You can do this in the macOS *Terminal* for each session:

- Run this script [Tools/mac_set_ulimit.sh](https://github.com/PX4/Firmware/blob/master/Tools/mac_set_ulimit.sh), or
- Enter this command: 
        sh
        ulimit -S -n 300

### macOS Catalina: Problem running cmake

As of macOS Catalina 10.15.1 there may be problems when trying to build the simulator with *cmake*. If you have build problems on this platform then try run the following command in your terminal:

```sh
xcode-select --install
sudo ln -s /Library/Developer/CommandLineTools/SDKs/MacOSX.sdk/usr/include/* /usr/local/include/
```

### Failed to import Python packages

"Failed to import" errors when running the `make px4_sitl jmavsim` command indicates that some Python packages are not installed (where expected).

    Failed to import jinja2: No module named 'jinja2'
    You may need to install it using:
        pip3 install --user jinja2
    

If you have already installed these dependencies this may be because there is more than one Python version on the computer (e.g. Python 2.7.16 Python 3.8.3), and the module is not present in the version used by the build toolchain.

You should be able to fix this by explicitly installing the dependencies as shown:

    pip3 install --user pyserial empy toml numpy pandas jinja2 pyyaml pyros-genmsg packaging