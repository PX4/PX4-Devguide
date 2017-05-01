# 在Raspberry Pi（树莓派）上安装ROS


本文介绍如何在一个作为Pixhawk协同计算机的树莓派2上安装ROS-indigo。

## 准备

- 一个可以工作的树莓派，配有监视器、键盘或者配置好的SSH连接。
- 这份指南假定你已经在树莓派上安装好了Raspbian "JESSIE"，如果没有，[安装它](https://www.raspberrypi.org/downloads/raspbian/)或者[升级](http://raspberrypi.stackexchange.com/questions/27858/upgrade-to-raspbian-jessie)Raspbian Wheezy到Jessie。

## 安装

参照[指南](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi)安装ROS Indigo。注意：安装"ROS-Comm"版本，"Desktop"版本太过庞大。

### 安装可能遇到的错误

如果下载包（例如`sudo apt-get install ros-indigo-ros-tutorials`）时遇到错误"unable to locate package ros-indigo-ros-tutorials"，那么按照下面方法操作：

进入你的catkin工作空间（例如~/ros_catkin_ws），并修改包的名字

```sh
$ cd ~/ros_catkin_ws

$ rosinstall_generator ros_tutorials --rosdistro indigo --deps --wet-only --exclude roslisp --tar > indigo-custom_ros.rosinstall
```

接着，用wstool升级你的工作空间

```sh
$ wstool merge -t src indigo-custom_ros.rosinstall

$ wstool update -t src
```

最后 (仍然在工作空间文件夹), source并构建你的文件。

```sh
$ source /opt/ros/indigo/setup.bash

$ source devel/setup.bash

$ catkin_make
```
