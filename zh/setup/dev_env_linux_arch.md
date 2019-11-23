# ArchLinux上开发环境的搭建

固件仓库里已经提供了一个脚本[Tools/setup/arch.sh](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/Tools/setup/arch.sh)方便你快速地在你的平台上搭建PX4的开发环境

The script installs (by default) all tools to build PX4 (without RTPS) for NuttX targets and run simulation with *jMAVsim*. You can additionally install the *Gazebo* simulator by specifying the command line argument: `--gazebo`.

![Gazebo on Arch](../../assets/gazebo/arch-gazebo.png)

> **Note** The instructions have been tested on [Manjaro](https://manjaro.org/) (Arch based distribution) as it is much easier to set up than Arch Linux.

To get and run the scripts, do either of:

- [Download PX4 Source Code](../setup/building_px4.md) and run the scripts in place: 
        git clone https://github.com/PX4/Firmware.git
        source Firmware/Tools/setup/arch.sh

- Download just the needed scripts and then run them: 
        sh
        wget https://raw.githubusercontent.com/PX4/Firmware/master/Tools/setup/arch.sh
        wget https://raw.githubusercontent.com/PX4/Firmware/master/Tools/setup/requirements.txt
        source arch.sh

The script takes the following optional parameters:

- `--gazebo`: Add this parameter parameter to install Gazebo from the [AUR](https://aur.archlinux.org/packages/gazebo/). > **Note** Gazebo gets compiled from source. It takes some time to install and requires entering the `sudo` password multiple times (for dependencies).
- `--no-nuttx`: Do not install the NuttX/Pixhawk toolchain (i.e. if only using simulation).
- `--no-sim-tools`: Do not install jMAVSim/Gazebo (i.e. if only targeting Pixhawk/NuttX targets)