# MAVROS

[MAVROS](http://wiki.ros.org/mavros#mavros.2BAC8-Plugins.sys_status)ROS包允许在运行ROS的计算机、支持MAVLink的飞控板以及支持MAVLink的地面站之间通讯。虽然MAVROS可以用来与任何支持MAVLink的飞控板通讯，但是本文仅就PX4飞行栈与运行ROS的协同计算机之间的通讯予以说明。

## 安装

MAVROS可以通过源文件或者二进制文件安装。推荐使用源文件安装。

### 二进制文件安装（Debian / Ubuntu）

从v0.5开始有x86和amd64平台的预编译的Debian安装包，从v0.9版本开始有Ubuntu armhf平台上的ARMv7安装包。

使用`apt-get`安装即可：

```sh
$ sudo apt-get install ros-indigo-mavros ros-indigo-mavros-extras ros‐indigo‐control‐toolbox
```

### 源文件安装

**依赖**

假定你有一个catkin工作空间位于`~/catkin_ws`，如果没有，则创建一个：

```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin init
```

安装MAVROS需要用到ROS python工具`wstool`，`rosinstall`和`catkin_tools`，尽管在安装ROS时可能已经安装过这些工具，但是仍然可以重新安装一遍：

```sh
$ sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools
```

注意，尽管可以使用catkin_make构建这些包，但是推荐使用catkin_tools，因为它更加友好，功能更加全面。

如果这是第一次使用wstool，那么需要初始化你的源空间：

```sh
$ wstool init ~/catkin_ws/src
```

现在准备构建

```sh
    # 1. get source (upstream - released)
$ rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
    # alternative: latest source
$ rosinstall_generator --upstream-development mavros | tee /tmp/mavros.rosinstall

    # 2. get latest released mavlink package
    # you may run from this line to update ros-*-mavlink package
$ rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall

    # 3. Setup workspace & install deps
$ wstool merge -t src /tmp/mavros.rosinstall
$ wstool update -t src
$ rosdep install --from-paths src --ignore-src --rosdistro indigo -y

    # finally - build
$ catkin build
```


> **提示:** 如果在树莓派上安装MAVROS，当运行`rosdep install ...`时可能会遇到和操作系统有关的错误。在rosdep命令中加上`--os=OS_NAME:OS_VERSION `，其中OS_NAME是你的操作系统名称，OS_VERSION是你的操作系统版本（例如：--os=debian:jessie）
