# Windows 平台基于虚拟机的工具链

> **注意：** [Windows Cygwin 工具链](../setup/dev_env_windows_cygwin.md) 是Windows平台唯一获得官方支持的开发环境。

Windows 平台开发者可以在运行Linux的虚拟机中运行 PX4 工具链。 设置好虚拟机后，在虚拟机内进行 PX4 开发环境的安装、设置的流程与原生 Linux 电脑没有任何差别。

> **提示：**尽量为虚拟机分配尽更多的 cpu 内核和内存资源。

虽然使用虚拟机进行开发环境的安装、测试非常简单，但用户们仍应知晓：

1. Firmware building will be slower than native building on Linux.
2. The JMAVSim frame rate be much slower than on native Linux. In some cases the vehicle may crash due to issues related to insufficient VM resources.
3. Gazebo and ROS can be installed, but are unusably slow.

## Instructions

There are multiple ways to setup a VM which is capable of executing the PX4 environment on your system. This guide walks you through a VMWare setup. VMWare performance is acceptable for basic usage (building Firmware) but not for running ROS or Gazebo.

1. Download [VMWare Player Freeware](https://www.vmware.com/products/workstation-player/workstation-player-evaluation.html)
2. Install it on your Windows system
3. Download the desired version of [Ubuntu Desktop ISO Image](https://www.ubuntu.com/download/desktop). (see [Linux Instructions Page](../setup/dev_env_linux.md) for recommended Ubuntu version).
4. Open *VMWare Player* and select the option to create a new virtual machine
5. In the VM creation wizard choose the downloaded Ubuntu ISO image as your installation medium and will automatically detect the operating system you want to use
6. Also in the wizard, select the resources you want to allocate to your virtual machine while it is running. Allocate as much memory and as many CPU cores as you can without rendering your host Windows system unusable.
7. Run your new VM at the end of the wizard and let it install Ubuntu following the setup instructions. Remember all settings are only for within your host operating system usage and hence you can disable any screen saver and local workstation security features which do not increase risk of a network attack.
8. Once the new VM is booted up make sure you install *VMWare tools drivers and tools extension* inside your guest system. This will enhance performance and usability of your VM usage: 
    - Significantly enhanced graphics performance
    - Proper support for hardware device usage like USB port allocation (important for target upload), proper mouse wheel scrolling, sound suppport
    - Guest display resolution adaption to the window size
    - Clipboard sharing to host system
    - File sharing to host system
9. Continue with [PX4 environment setup for Linux](../setup/dev_env_linux.md)