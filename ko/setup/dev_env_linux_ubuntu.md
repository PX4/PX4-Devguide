# 우분투 LTS / 데비안 리눅스 개발 환경

PX4 개발용 지원/시험 리눅스 운영체제 버전은 [우분투 리눅스 장기 지원 버전](https://wiki.ubuntu.com/LTS) 18.04 (Bionic Beaver)와 20.04(Focal Fossa)입니다. 두 버전에서는 [대부분의 PX4 대상](../setup/dev_env.md#supported-targets)(NuttX 기반 하드웨어, *퀄컴 스냅드래곤 플라이트* 하드웨어, 리눅스 기반 하드웨어, 모의시험 환경)을 빌드할 수 있습니다.

제각기 다른 대상 플랫폼에 대해 개발 환경 설치를 용이하게 하는 배시 스크립트를 제공해드립니다:

* **[ubuntu.sh](https://github.com/PX4/PX4-Autopilot/blob/{{ book.px4_version }}/Tools/setup/ubuntu.sh)**: Installs [Gazebo 9](../simulation/gazebo.md) and [jMAVSim](../simulation/jmavsim.md) simulators and/or [NuttX/Pixhawk](../setup/building_px4.md#nuttx) tools. [FastRTPS](#fast_rtps) 의존 요소는 넣지 마십시오.
* **[ubuntu_sim_ros_melodic.sh](https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/ubuntu_sim_ros_melodic.sh)**: [ROS "Melodic"](#rosgazebo)과 PX4를 우분투 18.04 LTS (이상) 에 설치합니다.

> **Tip** 스크립트를 *초기 상태의* 18.04 LTS 및 20.04 LTS 설치 기반에서 테스트했습니다. 기존 시스템에 "얹어 설치"하거나, 다른 우분투 출시판을 기반으로 설치할 경우 동작을 하지 않을*수 있습니다*.

아래 설명을 통해 스크립트 다운로드 및 활용법을 설명합니다.

<a id="sim_nuttx"></a>

## Gazebo, JMAVSim and NuttX (Pixhawk) Targets

Use the [ubuntu.sh](https://github.com/PX4/PX4-Autopilot/blob/{{ book.px4_version }}/Tools/setup/ubuntu.sh) script to set up a development environment that includes [Gazebo 9](../simulation/gazebo.md) and [jMAVSim](../simulation/jmavsim.md) simulators, and/or the [NuttX/Pixhawk](../setup/building_px4.md#nuttx) toolchain.

To install the toolchain:

1. [PX4 소스 코드를 다운로드하십시오](../setup/building_px4.md): 
        bash
        git clone https://github.com/PX4/PX4-Autopilot.git --recursive

2. 어떤 인자값도 주지 말고 **ubuntu.sh** 파일을 (배시 셸에서) 실행하여 모든 요소를 설치하십시오: 
        bash
        bash ./Tools/setup/ubuntu.sh
    
      
    * 스크립트 진행 과정 중 프롬프트에 응답하십시오.
    * nuttx 또는 모의시험 도구 설치를 생략할 때 `--no-nuttx` 와 `--no-sim-tools` 인자를 활용할 수 있습니다.
3. 설치가 끝나면 컴퓨터를 다시 시작하십시오.

> **Note** You can alternatively download [ubuntu.sh](https://github.com/PX4/PX4-Autopilot/blob/{{ book.px4_version }}/Tools/setup/ubuntu.sh) and [requirements.txt](https://github.com/PX4/PX4-Autopilot/blob/{{ book.px4_version }}/Tools/setup/requirements.txt) from the PX4 source repository (**/Tools/setup/**) and run ubuntu.sh in place:   
> `wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/{{ book.px4_version }}/Tools/setup/ubuntu.sh`   
> `wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/{{ book.px4_version }}/Tools/setup/requirements.txt`   
> `bash ubuntu.sh`

Notes:

* PX4는 가제보 버전 7, 8, 9에서 동작합니다. 가제보 9을 설치할 때 [gazebosim.org 과정](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)을 활용합니다.
* ROS를 다루려 할 경우 [ROS/가제보](#rosgazebo) 의 절차를 따르십시오(해당 설명 과정은 ROS 설치시 일부 구성요소로 가제보를 자동으로 설치합니다).
* 아래와 같이 GCC 버전이 나타남을 확인하면 NuttX 설치가 끝났음을 검증할 수 있습니다:
    
    ```bash
    $arm-none-eabi-gcc --version
    
    arm-none-eabi-gcc (GNU Tools for Arm Embedded Processors 7-2017-q4-major) 7.2.1 20170904 (release) [ARM/embedded-7-branch revision 255204]
    Copyright (C) 2017 Free Software Foundation, Inc.
    This is free software; see the source for copying conditions.  There is NO
    warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
    ```

<!-- Do we need to add to our scripts or can we assume correct version installs over?
Remove any old versions of the arm-none-eabi toolchain.</p>

<pre><code class="sh">sudo apt-get remove gcc-arm-none-eabi gdb-arm-none-eabi binutils-arm-none-eabi gcc-arm-embedded
sudo add-apt-repository --remove ppa:team-gcc-arm-embedded/ppa
</code></pre>

<p>-->

<a id="raspberry-pi-hardware"></a>

## Raspberry Pi

<!-- NOTE: RaPi docker toolchain (for comparison) here: https://github.com/PX4/containers/blob/master/docker/Dockerfile_armhf -->

To get the build toolchain for Raspberry Pi:

1. Download [ubuntu.sh](https://github.com/PX4/PX4-Autopilot/blob/{{ book.px4_version }}/Tools/setup/ubuntu.sh) and [requirements.txt](https://github.com/PX4/PX4-Autopilot/blob/{{ book.px4_version }}/Tools/setup/requirements.txt) from the PX4 source repository (**/Tools/setup/**):   
    `wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/{{ book.px4_version }}/Tools/setup/ubuntu.sh`   
    `wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/{{ book.px4_version }}/Tools/setup/requirements.txt`
2. 터미널에서 **ubuntu.sh** 명령을 실행하여 일반 의존 요소를 받으십시오: 
        bash
        bash ubuntu.sh --no-nuttx --no-sim-tools

3. 그 다음 ARMv7 교차-컴파일러(GCC 또는 clang)를 다음에 설명하는대로 설치하십시오.

### GCC

The official Raspberry Pi toolchains are not supported as PX4 has requires C++14 (which they do not support).

Ubuntu provides a set of pre-compiled toolchains that you can use instead. Install these with the terminal command:

    sudo apt-get install -y gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf
    

These package contains GCC/G++ 7.4.0 at time of writing. To test the toolchain, please execute:

    arm-linux-gnueabihf-gcc -v
    arm-linux-gnueabihf-g++ -v
    

### Clang

First [install GCC](#gcc) (needed to use clang).

We recommend you to get clang from the Ubuntu software repository as follows:

    sudo apt-get install clang
    

Example below for building PX4 firmware out of tree, using CMake.

```sh
cd <PATH-TO-PX4-SRC>
mkdir build/px4_raspberrypi_default_clang
cd build/px4_raspberrypi_default_clang
cmake \
-G"Unix Makefiles" \
-DCONFIG=px4_raspberrypi_default \
-UCMAKE_C_COMPILER \
-DCMAKE_C_COMPILER=clang \
-UCMAKE_CXX_COMPILER \
-DCMAKE_CXX_COMPILER=clang++ \
../..
make
```

### 자체 빌드

Additional developer information for using PX4 on Raspberry Pi (including building PX4 natively) can be found here: [Raspberry Pi 2/3 Navio2 Autopilot](https://docs.px4.io/master/en/flight_controller/raspberry_pi_navio2.html).

<a id="rosgazebo"></a>

## ROS/Gazebo

This section explains how to install [ROS/Gazebo](../ros/README.md) ("Melodic") for use with PX4.

To install the development toolchain:

1. 배시 셸에서 스크립트를 다운로드하십시오:   
    `wget https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/ubuntu_sim_ros_melodic.sh`
2. 스크립트를 실행하십시오: 
        bash
        bash ubuntu_sim_ros_melodic.sh 스크립트 처리 진행시 일부 프롬프트에 응답해야 합니다.

Note:

* ROS Melodic은 가제보 9에 기본적으로 설치합니다.
* catkin (ROS 빌드 시스템) 작업 환경은 **~/catkin_ws/**에 만듭니다.
* 스크립트는 ROS 위키 "Melodic" [우분투 페이지](http://wiki.ros.org/melodic/Installation/Ubuntu)의 설치 과정을 따릅니다.

## 스냅드래곤 비행체

Setup instructions for Snapdragon Flight are provided in the *PX4 User Guide*:

* [개발 환경](https://docs.px4.io/master/en/flight_controller/snapdragon_flight_dev_environment_installation.html)
* [소프트웨어 설치](https://docs.px4.io/master/en/flight_controller/snapdragon_flight_software_installation.html)
* [설정](https://docs.px4.io/master/en/flight_controller/snapdragon_flight_configuration.html)

<a id="fast_rtps"></a>

## Fast RTPS installation

[eProsima Fast RTPS](http://eprosima-fast-rtps.readthedocs.io/en/latest/) is a C++ implementation of the RTPS (Real Time Publish Subscribe) protocol. FastRTPS is used, via the [RTPS/ROS2 Interface: PX4-FastRTPS Bridge](../middleware/micrortps.md), to allow PX4 uORB topics to be shared with offboard components.

Follow the instructions in [Fast RTPS Installation](../setup/fast-rtps-installation.md) to install it.

## 추가 도구

After setting up the build/simulation toolchain, see [Additional Tools](../setup/generic_dev_tools.md) for information about other useful tools.

## 다음 단계

Once you have finished setting up the environment, continue to the [build instructions](../setup/building_px4.md).