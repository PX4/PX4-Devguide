# Windows Virtual Machine-Hosted Toolchain

Windows developers can run the PX4 toolchain in a virtual machine with linux as the guest operating system. After setting up the virtual machine the installation and setup of PX4 within the VM is exactly the same as on a native Linux computer.

> **Tip** Allocate as many CPU cores and memory resources to the VM as possible.

While using a VM is a very easy way to set up and test an environment for building firmware, users should be aware:
1. Firmware building will be slower than native building on Linux.
1. The JMAVSim frame rate be much slower than on native Linux. In some cases the vehicle may crash due to issues related to insufficient VM resources.
1. Gazebo and ROS can be installed, but are unusably slow.

## Instructions

There are multiple ways to setup a VM which is capable of executing the PX4 environment on your system. This guide walks you through a VMWare setup which showed usable performance for basic usage not including ROS usage and high resolution heavy graphics applications.

1. Download [VMWare Player Freeware](https://www.vmware.com/products/workstation-player/workstation-player-evaluation.html)
1. Install it on your Windows system
1. Download the desired version of [Ubuntu Desktop ISO Image](https://www.ubuntu.com/download/desktop) To determine the correct version consult the recommendations on the [Linux Instructions Page](../setup/dev_env_linux.md).
1. Open VMWare Player and select the option to create a new virtual machine
1. In the VM creation wizard choose the downloaded Ubuntu ISO image as your installation medium and will automatically detect the operating system you want to use
1. Also in the wizard select the resources you want to allocate to your virtual machine while it is running. It's recommended to allocate as much memory and CPU cores as you can without resulting in your host Windows system to get unusable at the same time.
1. Run your new VM at the end of the wizard and let it install Ubuntu following the setup instructions. Remember all settings are only for within your host operating system usage and hence you can disable any screen saver and local workstation security features which do not increase risk of a network attack.
1. Once the new VM is booted up make sure you install VMWare tools drivers and tools extension inside your guest system which will enhance performance and usability of your VM usage. Proper VMWare tools setup will enable e.g.:
    * Significantly enhanced graphics performance
    * Proper support for hardware device usage like USB port allocation (important for target upload), proper mouse wheel scrolling, sound suppport
    * Guest display resolution adaption to the window size
    * Clipboard sharing to host system
    * File sharing to host system
1. Continue with [PX4 environment setup for Linux](../setup/dev_env_linux.md)
