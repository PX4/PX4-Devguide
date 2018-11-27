# Windows 설치 방법

> **Warning** 윈도우 툴체인이 있긴하지만, 공식적으로 지원되는 것이 아니라 사용을 추천하지 않습니다. 펌웨어를 컴파일에 드는 시간이 오래 걸리고 Snapdragon Flight 같은 새로운 보드에 대해서 지원하지 않습니다. 많은 개발자가 컴퓨터 비전과 네비게이션 프로토타입으로 사용하는 표준 ROS 패키지를 사용할 수 없습니다. Windows에서 개발하기 전에, [Ubuntu](http://ubuntu.com)로 듀얼부트되는 환경을 고려해 보세요.

## 툴체인 설치

There are a number of ways you can set up a Windows development toolchain for PX4.

* The native toolchain allows you to build for NuttX/Pixhawk and jMAVSim simulator targets.
* The Windows Bash toolchain allows you to build for NuttX/Pixhawk targets (only).

### Native 툴체인

Download and install these on your system:

  * [PX4 Toolchain Installer v14 for Windows Download](http://firmware.diydrones.com/Tools/PX4-tools/px4_toolchain_installer_v14_win.exe) (32/64 bit systems, complete build system, drivers)
  * [PX4 USB Drivers](http://pixhawk.org/static/px4driver.msi) (32/64 bit systems)

### Windows에서 Bash (NEW!)

Windows users can alternatively install a *slightly modified* Ubuntu Linux PX4 development environment within [Bash on Windows](https://github.com/Microsoft/BashOnWindows), and use it to build firmware for NuttX/Pixhawk targets.
We have provided a script below that makes this easy.

> **Note** This approach does not currently support simulation because *Bash on Windows* does not enable Linux UI applications.

<span></span>
> **Tip** The script has been updated to  [install Fast RTPS from (Linux) binaries](../setup/fast-rtps-installation.md#linux).


To use the build script:
1. Install [Bash on Windows](https://github.com/Microsoft/BashOnWindows).
1. Download the <strong><a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/windows_bash_nuttx.sh" target="_blank" download>windows_bash_nuttx.sh</a></strong> script.
1. Open the bash shell and navigate to the directory containing the script.
1. Run the script using the command below (acknowledging any prompts as required):
  ```sh
  source windows_bash_nuttx.sh
  ```
1. Test the script by building the firmware:
  ```
  cd $src/Firmware
  make px4_fmu-v2_default
  ```
  On successful completion you'll find the firmware here: `Firmware/build/px4_fmu-v2/px4_fmu-v2_default.px4`
1. You can flash the custom firmware on Windows using *QGroundControl* or *Mission Planner* (it is not yet possible to directly flash the firmware from within the bash shell).

#### Build script details

The <a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/windows_bash_nuttx.sh">windows_bash_nuttx.sh</a> build script modifies the Ubuntu build instructions to remove Ubuntu-specific and UI-dependent components, including the *Qt Creator* IDE and the simulators.

In addition, it uses a [64 bit arm-none-eabi compiler](https://github.com/SolinGuo/arm-none-eabi-bash-on-win10-.git)
since BashOnWindows doesn't run 32 bit ELF programs (and the default compiler from `https://launchpad.net/gcc-arm-embedded` is 32 bit).

To add this compiler to your environment manually:

1. Download the compiler:
   ```sh
   wget https://github.com/SolinGuo/arm-none-eabi-bash-on-win10-/raw/master/gcc-arm-none-eabi-5_4-2017q2-20170512-linux.tar.bz2
   ```
1. Unpack it using this command line in the Bash On Windows console:
   ```sh
   tar -xvf gcc-arm-none-eabi-5_4-2017q2-20170512-linux.tar.bz2
   ```
   This will unpack the arm gcc cross-compiler to:
   ```
   gcc-arm-none-eabi-5_4-2017q2/bin
   ```
1. Add the to the environment (add the line to your bash profile to make the change permanent)
   ```
   export PATH=$HOME/gcc-arm-none-eabi-5_4-2017q2/bin:\$PATH
   ```


<!-- import docs for other tools and next steps. -->
{% include "_addition_dev_tools.txt" %}
