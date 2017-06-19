# 리눅스 개발 환경

Linux 배포판 중에 Debian / Ubuntu LTS가 기준입니다. 하지만 Cent OS나 Arch Linux을 위해 [boutique distribution instructions](../setup/dev_env_linux_boutique.md)도 사용 가능합니다.

## Permission 셋업

> **Warning** sudo'를 이용해서 permission 문제를 해결하지 마세요. 이로 인해 더 많은 permission 문제가 생길수도 있고 시스템을 다시 설치해야할수도 있습니다.

user는 "dialout" 그룹에 속해야 합니다. :

```sh
sudo usermod -a -G dialout $USER
```

다음으로 logout을 하고 다시 login합니다. 새로 login해야만 변경사항에 적용됩니다.

## 설치

패키지 목록을 업데이트하고 모든 PX4 빌드 타겟을 위해 필요한 의존 패키지를 설치합니다. PX4가 지원하는 4가지 계열 :

* NuttX 기반 하드웨어: [Pixhawk](../flight_controller/pixhawk.md), [Pixfalcon](../flight_controller/pixfalcon.md),
  [Pixracer](../flight_controller/pixracer.md), [Pixhawk 3 Pro](../flight_controller/pixhawk3_pro.md), [Crazyflie](../flight_controller/crazyflie2.md),
  [Intel® Aero Ready to Fly Drone](../flight_controller/intel_aero.md)
* [Qualcomm Snapdragon Flight hardware](../flight_controller/snapdragon_flight.md)
* Linux 기반 하드웨어: [Raspberry Pi 2/3](../flight_controller/raspberry_pi_navio2.md), Parrot Bebop
* 호스트 시뮬레이션: [jMAVSim SITL](../simulation/sitl.md) 와 [Gazebo SITL](../simulation/gazebo.md)

> **Info** Make보다 빠르게 빌드하기 위해 [Ninja 빌드 시스템](../setup/dev_env_linux_boutique.md#ninja-build-system)을 설치합니다. 이미 설치되어 있다면 자동으로 선택해서 빌드하게 됩니다.

```sh
sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
sudo apt-get update
sudo apt-get install python-argparse git-core wget zip \
    python-empy qtcreator cmake build-essential genromfs -y
# simulation tools
sudo apt-get install ant protobuf-compiler libeigen3-dev libopencv-dev openjdk-8-jdk openjdk-8-jre clang-3.5 lldb-3.5 -y
# required python packages
sudo apt-get install python-pip
sudo -H pip install pandas jinja2
```

### NuttX 기반 하드웨어

Ubuntu에 기본으로 serial modem manager가 설치되어 있어서 로보틱스 관련해서 시리얼 포트나 \( USB 시리얼\) 을 사용하는 경우 방해가 됩니다. 효과적으로 삭제하는 방법은 :

```sh
sudo apt-get remove modemmanager
```

패키지 목록을 업데이트하고 의존 패키지를 설치합니다. 특정 버전이 필요한 패키지는 해당 버전의 패키지를 설치해야만 합니다.

```sh
sudo apt-get install python-serial openocd \
    flex bison libncurses5-dev autoconf texinfo build-essential \
    libftdi-dev libtool zlib1g-dev \
    python-empy  -y
```

arm-none-eabi 툴체인을 추가하기 전에 이전에 설치된 것을 제거되었는지 확인합니다.

```sh
sudo apt-get remove gcc-arm-none-eabi gdb-arm-none-eabi binutils-arm-none-eabi gcc-arm-embedded
sudo add-apt-repository --remove ppa:team-gcc-arm-embedded/ppa
```

4.9나 5.4 버전의 arm-none-eabi 툴체인을 수동으로 설치하려면 [툴체인 설치 방법](../setup/dev_env_linux_boutique.md#toolchain-installation))을 참고합니다.

### Snapdragon Flight

#### 툴체인 설치

```sh
sudo apt-get install android-tools-adb android-tools-fastboot fakechroot fakeroot unzip xz-utils wget python python-empy -y
```

툴체인 설치는 https://github.com/ATLFlight/cross_toolchain 방법을 따라합니다.

새로운 설정을 로드합니다:

```sh
source ~/.bashrc
```

#### Sysroot 설치

sysroot은 Snapdragon Flight 어플리케이션 프로세서용 크로스 컴파일 어플리케이션에 필요한 라이브러리와 헤더 파일을 제공하는데 필요합니다.

qrlSDK sysroot은 카메라, GPU 등에 필요한 헤더 파일과 라이브러리를 제공합니다.

[Flight\_3.1.1\_qrlSDK.zip](http://support.intrinsyc.com/attachments/download/690/Flight_3.1.1_qrlSDK.zip) 파일을 다운로드 받고 `cross_toolchain/download/`에 저장합니다.

```sh
cd cross_toolchain
unset HEXAGON_ARM_SYSROOT
./qrlinux_sysroot.sh
```

다음을 ~/.bashrc 에 추가 :

```sh
export HEXAGON_ARM_SYSROOT=${HOME}/Qualcomm/qrlinux_v3.1.1_sysroot
```

새로운 설정을 로드:

```sh
source ~/.bashrc
```

sysroot 옵션은 [Sysroot 설치](https://github.com/ATLFlight/cross_toolchain/blob/sdk3/README.md#sysroot-installation)를 참고하세요.

#### ADSP 펌웨어 업데이트

빌드, 플래쉬, 실행하기 전에 [ADSP 펌웨어](../flight_controller/snapdragon_flight_advanced.md#updating-the-adsp-firmware) 업데이트가 필요합니다.

#### 레퍼런스

Snapdragon Flight 툴체인, SW 셋업과 검증에 관련된 외부 문서 :
[ATLFlightDocs](https://github.com/ATLFlight/ATLFlightDocs/blob/master/README.md)

DSP에서 받은 메시지는 mini-dm을 사용해서 볼 수 있습니다.

```sh
${HEXAGON_SDK_ROOT}/tools/debug/mini-dm/Linux_Debug/mini-dm
```

Note: 특히 Mac에서는 [nano-dm](https://github.com/kevinmehall/nano-dm)을 사용할 수 있습니다.

### Raspberry Pi 하드웨어

라즈베리파이 하드웨어로 개발하는 경우 ARMv7 cross-compiler로 GCC나 clang을 다운로드해야합니다.
추천하는 툴체인은 GCC 4.8.3으로 `https://github.com/raspberrypi/tools.git`에서 클론할 수 있습니다.
`PATH` 환경 변수는 `arm-linux-gnueabihf-` 접두어를 가진 도구(gcc, g++, strip등)의 gcc 크로스 컴파일러에 대한 path를 포함하고 있어야만 합니다.

```sh
git clone https://github.com/raspberrypi/tools.git ${HOME}/rpi-tools

# 컴파일러 테스트하기
$HOME/rpi-tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-gcc -v

# ~/.profile를 수정해서 PATH 변수 업데이트
echo 'export PATH=$PATH:$HOME/rpi-tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin' >> ~/.profile

# 이번 세션만 적용되도록 PATH 변수 업데이트
export PATH=$PATH:$HOME/rpi-tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin
```

#### clang

clang을 사용하기 위해서 GCC를 사용해야 합니다.

[LLVM 다운로드](http://releases.llvm.org/download.html)에서 배포하는 clang을 다운로드 받아서 압축을 풉니다. `CLANG_DIR` 경로에 압축을 풀었다고 가정하고 `clang` 바이너리는 `CLANG_DIR/bin`에 위치하게 됩니다. `GCC_DIR`에 GCC 크로스 컴파일러가 위치합니다. `GCC_DIR` bin 디렉토리로 clang에 대한 symlink를 셋업하고 `GCC_DIR/bin`를 `PATH`에 추가합니다.

아래 예제에서 PX4 펌웨어를 빌드하는데 CMake를 사용합니다.
```sh
ln -s <CLANG_DIR>/bin/clang <GCC_DIR>/bin/clang
ln -s <CLANG_DIR>/bin/clang++ <GCC_DIR>/bin/clang++
export PATH=<GCC_DIR>/bin:$PATH

cd <PATH-TO-PX4-SRC>
mkdir build_posix_rpi_cross_clang
cd build_posix_rpi_cross_clang
cmake \
-G"Unix Makefiles" \
-DCONFIG=posix_rpi_cross \
-DCMAKE_C_COMPILER=clang \
-DCMAKE_CXX_COMPILER=clang++ \
..

```

### 패롯 비밥

패롯 비밥으로 개발하는 경우 RPi 리눅스 툴체인을 설치해야만 합니다. [Raspberry Pi 하드웨어](../flight_controller/raspberry_pi_navio2.md)에 있는 설명을 참고하세요.

다음으로 ADB를 설치합니다.

``sh
sudo apt-get install android-tools-adb -y` ``

## 마무리

이제 [처음 빌드하기](../setup/building_px4.md)를 이어서 실행하면 됩니다!
