# Windows Installation Instructions

> **Warning** Although a Windows toolchain is available, it is not officially supported (and we discourage its use). It is unbearably slow during Firmware compilation and does not support new boards like Snapdragon Flight. It also cannot run the standard robotics software packages many developers use to prototype computer vision and navigation. Before starting to develop on Windows, consider installing a dual-boot environment with [Ubuntu](http://ubuntu.com).

## Toolchain Installation

There are a number of ways you can set up a Windows development toolchain for PX4.

* The native toolchain allows you to build for NuttX/Pixhawk and jMAVSim simulator targets. 
* The Windows Bash toolchain allows you to build for NuttX/Pixhawk targets (only). 

### Native toolchain

Download and install these on your system:

  * [PX4 Toolchain Installer v14 for Windows Download](http://firmware.diydrones.com/Tools/PX4-tools/px4_toolchain_installer_v14_win.exe) (32/64 bit systems, complete build system, drivers)
  * [PX4 USB Drivers](http://pixhawk.org/static/px4driver.msi) (32/64 bit systems)

### Bash on Windows (NEW!)

Windows users can alternatively install a *slightly modified* Ubuntu Linux PX4 development environment within [Bash on Windows](https://github.com/Microsoft/BashOnWindows), and use it to build firmware for NuttX/Pixhawk targets. 
We have provided a script below that makes this easy.

> **Note** This approach does not currently support simulation because *Bash on Windows* does not enable Linux UI applications.
  

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
  make px4fmu-v2_default
  ```
  On successful completion you'll find the firmware here: `Firmware/build_px4fmu-v2/src/firmware/nuttx/px4fmu-v2_default.px4`
1. You can flash the custom firmware on Windows using *QGroundControl* or *Mission Planner* (it is not yet possible to directly flash the firmware from within the bash shell).

#### Build script details

The <a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/windows_bash_nuttx.sh">windows_bash_nuttx.sh</a> build script modifies the Ubuntu build instructions to removes Ubuntu-specific and UI-dependent components, including the *Qt Creator* IDE and the simulators. 

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