# 在 Ubuntu 上安装英特尔 RealSense R200 的驱动程序

本教程旨在提供有关如何在 linux 环境中安装英特尔实感 r200 相机头的相机驱动程序的说明, 以便可以通过机器人操作系统 (ros) 访问收集到的图像。 实感 r200 相机头如下图所示:

![](../../assets/realsense_intel/realsense.png)

驱动程序包的安装是在 Virtual Box 中作为虚拟机运行的 ubuntu 操作系统 (os) 上执行的。 运行 Virtual Box 的宿主机、虚拟机的规格如下:

- 主机操作系统：Windows 8
- 处理器：Intel(R) Core(TM) i7-4702MQ CPU @ 2.20GHz
- Virtual Box：Oracle VM。 版本 5.0.14 r105127
- 扩展：安装了 Virtual Box 的扩展包（用于 USB3.0 支持）
- 客户机操作系统：linux-ubuntu 14.04.3 LTS

本教程按以下方式排序: 在第一部分中, 演示如何在 Virtual Box 中安装 ubuntu 14.04 作为客户机系统。 第二部分会演示如何安装 ROS Indigo 和相机驱动程序。 随后频繁使用的短语示意如下：

- 虚拟框（VB）：运行不同虚拟机的程序。 此处使用 Oracle 虚拟机。
- 虚拟机（VM）：作为来宾系统在虚拟框中运行的操作系统。 此处使用 Ubuntu。

## 在虚拟机中安装 Ubuntu 14.04.3 LTS

- 创建新的虚拟机 (vm): linux 64位。
- 下载 ubuntu 14.04.3 lts 的 iso 文件: ([ubuntu-14.04.3-desktop-amd64.iso](http://www.ubuntu.com/download/desktop))。
- Ubuntu 的安装: 
  - 在安装过程中，保留以下两项: 
    - 安装时下载更新 
    - 安装此第三方软件
- After the installation you might need to enable the Virtual Box to display Ubuntu on the whole desktop: 
  - Start VM Ubuntu and login, Click on **Devices->Insert Guest Additions CD image** in the menu bar of the Virtual Box.
  - Click on **Run** and enter password on the windows that pop up in Ubuntu.
  - Wait until the installation is completed and then restart. Now, it should be possible to display the VM on the whole desktop.
  - If a window pops up in Ubuntu that asks whether to update, reject to update at this point.
- Enable USB 3 Controller in Virtual Box: 
  - Shut down Virtual Machine.
  - Go to the settings of the Virtual Machine to the menu selection USB and choose: "USB 3.0(xHCI)". This is only possible if you have installed the extension package for the Virtual Box.
  - Start the Virtual Machine again.

## 安装 ROS Indigo

- Follow instructions given at [ROS indigo installation guide](http://wiki.ros.org/indigo/Installation/Ubuntu): 
  - Install Desktop-Full version.
  - Execute steps described in the sections "Initialize rosdep" and "Environment setup".

## Installing camera driver

- Install git:

```bash
sudo apt-get install git
```

- Download and install the driver 
  - Clone [RealSense_ROS repository](https://github.com/bestmodule/RealSense_ROS): 
        bash
        git clone https://github.com/bestmodule/RealSense_ROS.git

- Follow instructions given in [here](https://github.com/bestmodule/RealSense_ROS/tree/master/r200_install).
  
  - Press the enter button when the questions whether to install the following installation packages show up:
    
        Intel Low Power Subsystem support in ACPI mode (MFD_INTEL_LPSS_ACPI) [N/m/y/?] (NEW)
        
    
        Intel Low Power Subsystem support in PCI mode (MFD_INTEL_LPSS_PCI) [N/m/y/?] (NEW)
        
        
    
        Dell Airplane Mode Switch driver (DELL_RBTN) [N/m/y/?] (NEW)
        
  
  - The following error message that can appear at the end of the installation process should not lead to a malfunction of the driver: ```rmmod: ERROR: Module uvcvideo is not currently loaded```

- After the installation has completed, reboot the Virtual Machine.

- Test camera driver:
  
  - Connect the Intel RealSense camera head with the computer with a USB3 cable that is plugged into a USB3 receptacle on the computer.
  - Click on Devices->USB-> Intel Corp Intel RealSense 3D Camera R200 in the menu bar of the Virtual Box, in order to forward the camera USB connection to the Virtual Machine.
  - Execute the file [unpacked folder]/Bin/DSReadCameraInfo: 
    - If the following error message appears, unplug the camera (physically unplug USB cable from the computer). Plug it in again + Click on Devices->USB-> Intel Corp Intel RealSense 3D Camera R200 in the menu bar of the Virtual Box again and execute again the file [unpacked folder]/Bin/DSReadCameraInfo. ```DSAPI call failed at ReadCameraInfo.cpp:134!```
    - If the camera driver works and recognises the Intel RealSense R200, you should see specific information about the Intel RealSense R200 camera head.

- Installation and testing of the ROS nodlet:
  
  - Follow the installation instructions in the "Installation" section given [here](https://github.com/bestmodule/RealSense_ROS/blob/master/realsense_dist/2.3/doc/RealSense-ROS-R200-nodelet.md), to install the ROS nodlet.
  - Follow the instructions in the "Running the R200 nodelet" section given [here](https://github.com/bestmodule/RealSense_ROS/blob/master/realsense_dist/2.3/doc/RealSense-ROS-R200-nodelet.md), to test the ROS nodlet together with the Intel RealSense R200 camera head. 
    - If everything works, the different data streams from the Intel RealSense R200 camera are published as ROS topics.