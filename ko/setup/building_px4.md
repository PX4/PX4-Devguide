# PX4 소프트웨어 빌드

PX4는 모의시험 환경과 하드웨어 타겟 모두에 대해 콘솔 또는 IDE 환경에서 빌드할 수 있습니다.

> **Note** 다음 절차를 따르기 전에 우선 [개발자 툴체인](../setup/dev_env.md)을 호스트 운영 체제와 타겟 하드웨어용으로 설치해야합니다.

<span></span>

> **Tip** 일반 빌드 문제에 대한 해결책은 하단의 [문제 해결](#troubleshooting) 부준을 참고하십시오.

<a id="get_px4_code"></a>

## Download the PX4 Source Code

The PX4 source code is stored on Github in the [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) repository. To get the *very latest* version onto your computer, enter the following command into a terminal:

```sh
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

> **Note** 이 방법이 최신 코드를 빌드하는데 필요한 모든 과정입니다. PX4에 기여할 목적의 더 많은 git 활용 내용은 [git 예제 > PX4에 코드 기여하기](../contribute/git_examples.md#contributing_code) 에 있습니다.

<a id="jmavsim_build"></a>

## First Build (Using the jMAVSim Simulator)

First we'll build a simulated target using a console environment. This allows us to validate the system setup before moving on to real hardware and an IDE.

Navigate into the **PX4-Autopilot** directory and start [jMAVSim](../simulation/jmavsim.md) using the following command:

```sh
make px4_sitl jmavsim
```

This will bring up the PX4 console below:

![PX4 Console (jMAVSim)](../../assets/toolchain/console_jmavsim.png)

The drone can be flown by typing:

```sh
pxh> commander takeoff
```

![jMAVSim UI](../../assets/toolchain/jmavsim_first_takeoff.png)

The drone can be landed by typing `commander land` and the whole simulation can be stopped by doing **CTRL+C** (or by entering `shutdown`).

Flying the simulation with the ground control station is closer to the real operation of the vehicle. Click on a location in the map while the vehicle is flying (takeoff flight mode) and enable the slider. This will reposition the vehicle.

![QGroundControl GoTo](../../assets/toolchain/qgc_goto.jpg)

> **Tip** [가제보(Gazebo) 모의시험 환경](../simulation/gazebo.md), [AirSim 모의시험 환경](../simulation/airsim.md)과 같은 다른 여러 [모의시험 환경](../simulation/README.md)에서도 PX4를 활용할 수 있습니다. 이들 역시 *make* 명령으로 시작합니다. 예를 들면: ```make px4_sitl gazebo```

<a id="nuttx"></a>

## NuttX / Pixhawk Based Boards

<a id="building_nuttx"></a>

### Building

To build for NuttX- or Pixhawk- based boards, navigate into the **PX4-Autopilot** directory and then call `make` with the build target for your board.

For example, to build for *Pixracer* you would use the following command:

```sh
cd PX4-Autopilot
make px4_fmu-v4_default
```

> **Note** 위 예제에서 빌드 타겟의 처음 부분인 `px4_fmu-v4`는 비행체 제어부 하드웨어 일부 기종용 펌웨어 이름이며, `default`는 설정 이름입니다 (이 경우 "default" 설정입니다). `default`는 선택 사항이기에, 대신 다음 명령을 실행할 수 있습니다: ```make px4_fmu-v4```

A successful run will end with similar output to:

```sh
-- Build files have been written to: /home/youruser/src/PX4-Autopilot/build/px4_fmu-v4_default
[954/954] Creating /home/youruser/src/PX4-Autopilot/build/px4_fmu-v4_default/px4_fmu-v4_default.px4
```

The following list shows the build commands for common boards:

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

A successful run will end with this output:

```sh
Erase  : [====================] 100.0%
Program: [====================] 100.0%
Verify : [====================] 100.0%
Rebooting.

[100%] Built target upload
```

## 기타 보드

The following boards have more complicated build and/or deployment instructions.

### 라즈베리 파이 2/3 보드

The command below builds the target for [Raspberry Pi 2/3 Navio2](https://docs.px4.io/master/en/flight_controller/raspberry_pi_navio2.html).

#### 교차 컴파일러 빌드

Set the IP (or hostname) of your RPi using:

```sh
export AUTOPILOT_HOST=192.168.X.X
```

or

```sh
export AUTOPILOT_HOST=pi_hostname.domain
```

> **Note** 환경 변수 값을 빌드 전에 설정하지 않으면, `make upload` 명령 실행시 라즈베리 파이 찾기에 실패합니다.

Build the executable file:

```sh
cd PX4-Autopilot
make emlid_navio2 # for cross-compiler build
```

The "px4" executable file is in the directory **build/emlid_navio2_default/**. Make sure you can connect to your RPi over ssh, see [instructions how to access your RPi](https://docs.px4.io/master/en/flight_controller/raspberry_pi_navio2.html#developer-quick-start).

Then upload it with:

```sh
cd PX4-Autopilot
make emlid_navio2 upload # for cross-compiler build
```

Then, connect over ssh and run it with (as root):

```sh
cd ~/px4
sudo ./bin/px4 -s px4.config
```

#### 자체 빌드

If you're building *directly* on the Pi, you will want the native build target (emlid_navio2_native).

```sh
cd PX4-Autopilot
make emlid_navio2_native # for native build
```

The "px4" executable file is in the directory **build/emlid_navio2_native/**. Run it directly with:

```sh
sudo ./build/emlid_navio2_native/px4 build/emlid_navio2_native/etc -s ./posix-configs/rpi/px4.config
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

To autostart px4, add the following to the file **/etc/rc.local** (adjust it accordingly if you use native build), right before the `exit 0` line:

```sh
cd /home/pi && ./bin/px4 -d -s px4.config > px4.log
```

### OcPoC-Zynq Mini

Build instructions for the [OcPoC-Zynq Mini](https://docs.px4.io/master/en/flight_controller/ocpoc_zynq.html) are covered in:

- [Aerotenna OcPoC-Zynq 미니 비행체 제어 장치 > OcPoC-Zynq용 PX4 빌드](https://docs.px4.io/master/en/flight_controller/ocpoc_zynq.html#building-px4-for-ocpoc-zynq)
- [OcPoC PX4 설정 페이지](https://aerotenna.readme.io/docs/px4-setup)

### QuRT / 스냅드래곤 기반 보드

This section shows how to build for the [Qualcomm Snapdragon Flight](https://docs.px4.io/master/en/flight_controller/snapdragon_flight.html).

#### 빌드

> **Note** (UART 기반) [퀄컴 전동 변속기 보드](http://shop.intrinsyc.com/products/qualcomm-electronic-speed-control-board)를 사용한다면, [이 곳](https://github.com/ATLFlight/ATLFlightDocs/blob/master/PX4.md) 절차를 따르십시오. PWM기반 일반 전동 변속기 보드를 사용한다면 이 페이지의 다음 과정을 계속 따르는 것이 좋습니다.

The commands below build the targets for the Linux and the DSP side. Both executables communicate via [muORB](../middleware/uorb.md).

```sh
cd PX4-Autopilot
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

Note that this will also copy (and overwrite) the two config files [mainapp.config](https://github.com/PX4/PX4-Autopilot/blob/master/posix-configs/eagle/flight/mainapp.config) and [px4.config](https://github.com/PX4/PX4-Autopilot/blob/master/posix-configs/eagle/flight/px4.config) to the device. Those files are stored under /usr/share/data/adsp/px4.config and /home/linaro/mainapp.config respectively if you want to edit the startup scripts directly on your vehicle.

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

The PX4 system supports Qt Creator, Eclipse and Sublime Text. Qt Creator is the most user-friendly variant and hence the only officially supported IDE. Unless an expert in Eclipse or Sublime, their use is discouraged. Hardcore users can find an [Eclipse project](https://github.com/PX4/PX4-Autopilot/blob/master/eclipse.project) and a [Sublime project](https://github.com/PX4/PX4-Autopilot/blob/master/Firmware.sublime-project) in the source tree.

{% youtube %}https://www.youtube.com/watch?v=Bkk8zttWxEI&rel=0&vq=hd720{% endyoutube %}

## Qt Creator 기능 

Qt creator offers clickable symbols, auto-completion of the complete codebase and building and flashing firmware.

![](../../assets/toolchain/qtcreator.png)

### 리눅스용 Qt Creator

Before starting Qt Creator, the [project file](https://gitlab.kitware.com/cmake/community/wikis/doc/cmake/Generator-Specific-Information#codeblocks-generator) needs to be created:

```sh
cd ~/src/PX4-Autopilot
mkdir ../Firmware-build
cd ../Firmware-build
cmake ../PX4-Autopilot -G "CodeBlocks - Unix Makefiles"
```

Then load the CMakeLists.txt in the root PX4-Autopilot folder via **File > Open File or Project** (Select the CMakeLists.txt file).

After loading, the **play** button can be configured to run the project by selecting 'custom executable' in the run target configuration and entering 'make' as executable and 'upload' as argument.

### Windows용 Qt Creator

> **Note** 윈도우에서는 Qt 크리에이터로 PX4 개발을 시험해보지 않았습니다.

### Mac OS용 Qt Creator

Before starting Qt Creator, the [project file](https://gitlab.kitware.com/cmake/community/wikis/doc/cmake/Generator-Specific-Information#codeblocks-generator) needs to be created:

```sh
cd ~/src/PX4-Autopilot
mkdir -p build/creator
cd build/creator
cmake ../.. -G "CodeBlocks - Unix Makefiles"
```

That's it! Start *Qt Creator*, then complete the steps in the video below to set up the project to build.

{% youtube %}https://www.youtube.com/watch?v=0pa0gS30zNw&rel=0&vq=hd720{% endyoutube %}

<a id="make_targets"></a>

## PX4 Make Build Targets

The previous sections showed how you can call *make* to build a number of different targets, start simulators, use IDEs etc. This section shows how *make* options are constructed and how to find the available choices.

The full syntax to call *make* with a particular configuration and initialization file is:

```sh
make [VENDOR_][MODEL][_VARIANT] [VIEWER_MODEL_DEBUGGER_WORLD]
```

**VENDOR_MODEL_VARIANT**: (also known as `CONFIGURATION_TARGET`)

- **VENDOR:** 보드의 제조사: `px4`, `aerotenna`, `airmind`, `atlflight`, `auav`, `beaglebone`, `intel`, `nxp` 등. 픽스호크 계열 보드 제조사 이름은 `px4` 입니다.
- **MODEL:** *보드* "모델": `sitl`, `fmu-v2`, `fmu-v3`, `fmu-v4`, `fmu-v5`, `navio2` 등.
- **VARIANT:** 개별 일부 설정을 나타냅니다. 예: `default` 설정에 일부 구성요소가 들어있지 않는 `rtps`, `lpe`. 대부분 `default`를 사용하며, 생략합니다.

> **Tip** 아래 명령으로 *모든* 가용 `CONFIGURATION_TARGET` 옵션을 확인해볼 수 있습니다: 
> 
>     sh
>       make list_config_targets

**VIEWER_MODEL_DEBUGGER_WORLD:**

- **VIEWER:** `gazebo`, `jmavsim`에 연결할 모의실험 환경("viewer") <!-- , ?airsim -->

- **MODEL:** 모의시험 환경에서 불러와서 활용할 *운송 수단* 모델(예: `iris` (*default*), `rover`, `tailsitter` 등). 적절한 매개변수를 선택하는 [시작 스크립트](..\simulation\README.md#scripts)에서 활용할 선택 모델을 `PX4_SIM_MODEL` 환경 변수로 설정합니다.

- **DEBUGGER:** 활용 디버거 `none` (*default*), `ide`, `gdb`, `lldb`, `ddd`, `valgrind`, `callgrind`. 자세한 내용은 [모의시험 환경 디버깅](../debug/simulation_debugging.md)을 살펴보십시오.
- **WORLD:** (가제보 전용). 불러온 월드([PX4/sitl_gazebo/worlds](https://github.com/PX4/sitl_gazebo/tree/master/worlds))를 설정합니다. 기본값은 [empty.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/empty.world)입니다. 자세한 내용은 [가제보 > 지정 월드 불러오기](../simulation/gazebo.md#set_world)를 살펴보십시오.

> **Tip** 아래 명령으로 *모든* 가용 `VIEWER_MODEL_DEBUGGER_WORLD` 옵션을 확인할 수 있습니다: 
> 
>     sh
>       make px4_sitl list_vmd_make_targets

Notes:

- `CONFIGURATION_TARGET` 변수와 `VIEWER_MODEL_DEBUGGER` 변수의 대부분의 값은 기본값이기에, 다른 값으로의 설정은 선택입니다. 예를 들어 `gazebo` 는 `gazebo_iris` 또는 `gazebo_iris_none`과 동일합니다. 
- 두개의 다른 설정 사이에 기본값을 지정하려면 밑줄 문자를 셋 사용할 수 있습니다. 예를 들면, `gazebo___gdb`는 `gazebo_iris_gdb`와 동일합니다.
- PX4를 시작한 후 모의시험 환경의 동작을 기다리려면 `VIEWER_MODEL_DEBUGGER`에 `none` 값을 사용할 수 있습니다. 예로, `make px4_sitl_default none` 명령을 사용하는 PX4와 `./Tools/jmavsim_run.sh -l` 명령을 사용하는 jMAVSim을 시작하려면:

The `VENDOR_MODEL_VARIANT` options map to particular *cmake* configuration files in the PX4 source tree under the [/boards](https://github.com/PX4/PX4-Autopilot/tree/master/boards) directory. Specifically `VENDOR_MODEL_VARIANT` maps to a configuration file **boards/VENDOR/MODEL/VARIANT.cmake** (e.g. `px4_fmu-v5_default` corresponds to [boards/px4/fmu-v5/default.cmake](https://github.com/PX4/PX4-Autopilot/blob/master/boards/px4/fmu-v5/default.cmake)).

Additional make targets are discussed in the following sections (list is not exhaustive):

<a id="bloaty_compare_master"></a>

### Binary Size Profiling

The `bloaty_compare_master` build target allows you to get a better understanding of the impact of changes on code size. When it is used, the toolchain downloads the latest successful master build of a particular firmware and compares it to the local build (using the [bloaty](https://github.com/google/bloaty) size profiler for binaries).

> **Tip** 이 과정을 통해 `px4_fmu-v2_default` 빌드 대상이 (아마도) 1MB 플래시 용량 제한에 걸리는 원인 변경을 분석할 수 있습니다.

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

<a id="firmware_version"></a>

## Firmware Version & Git Tags

The *PX4 Firmware Version* and *Custom Firmware Version* are published using the MAVLink [AUTOPILOT_VERSION](https://mavlink.io/en/messages/common.html#AUTOPILOT_VERSION) message, and displayed in the *QGroundControl* **Setup > Summary** airframe panel:

![Firmware info](../../assets/gcs/qgc_setup_summary_airframe_firmware.jpg)

These are extracted at build time from the active *git tag* for your repo tree. The git tag should be formatted as `<PX4-version>-<vendor-version>` (e.g. the tag in the image above was set to `v1.8.1-2.22.1`).

> **Warning** 다른 git 태그 형식을 취하면 버전 정보를 제대로 나타내지 못합니다.

<a id="troubleshooting"></a>

## Troubleshooting

### 일반 빌드 오류

Many build problems are caused by either mismatching submodules or an incompletely cleaned-up build environment. Updating the submodules and doing a `distclean` can fix these kinds of errors:

    git submodule update --recursive
    make distclean
    

### Flash overflowed by XXX bytes

The `region 'flash' overflowed by XXXX bytes` error indicates that the firmware is too large for the target hardware platform. This is common for `make px4_fmu-v2_default` builds, where the flash size is limited to 1MB.

If you're building the *vanilla* master branch, the most likely cause is using an unsupported version of GCC. In this case, install the version specified in the [Developer Toolchain](../setup/dev_env.md) instructions.

If building your own branch, it is possibly you have increased the firmware size over the 1MB limit. In this case you will need to remove any drivers/modules that you don't need from the build.

<a id="macos_open_files"></a>

### macOS: Too many open fileserror

MacOS allows a default maximum of 256 open files in all running processes. The PX4 build system opens a large number of files, so you may exceed this number.

The build toolchain will then report `Too many open files` for many files, as shown below:

```sh
/usr/local/Cellar/gcc-arm-none-eabi/20171218/bin/../lib/gcc/arm-none-eabi/7.2.1/../../../../arm-none-eabi/bin/ld: cannot find NuttX/nuttx/fs/libfs.a: Too many open files
```

The solution is to increase the maximum allowed number of open files (e.g. to 300). You can do this in the macOS *Terminal* for each session:

- Run this script [Tools/mac_set_ulimit.sh](https://github.com/PX4/PX4-Autopilot/blob/master/Tools/mac_set_ulimit.sh), or
- 다음 명령을 입력하십시오 
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