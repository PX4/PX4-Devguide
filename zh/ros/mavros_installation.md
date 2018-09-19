# MAVROS

[ mavros ](http://wiki.ros.org/mavros#mavros.2BAC8-Plugins.sys_status) 功能包提供了 一台能够运行ros的机载电脑、支持MAVLINK协议的飞控和支持MAVLINK的地面站这三者之间的通讯功能。

> ** 注意 *** MAVROS * 是 ROS 与 MAVLink 协议之间的 有"官方" 支持的ROS功能包。 它当前正在扩展以启用 [ fast-RTPS messaging ](../middleware/micrortps.md), 包括将 PX4 [ uORB 消息 ](../middleware/uorb.md) 转换为常见 ROS 话题的代码。

虽然 MAVROS 可用于与任何支持 MAVLink 协议的自动驾驶仪进行通信, 但此文档主要针对 PX4 飞控固件与运行ROS 的机载电脑之间的通讯问题。

## 安装

MAVROS功能包可以用源代码或二进制方式安装。（这是ROS功能包常见的两种安装方式，源代码安装可以修改源码，二进制方式安装则不行，只能直接调用源码） 建议有ROS基础的开发者使用源代码方式安装。

> ** 提示 **这些安装说明是 [ 官方安装指南 ](https://github.com/mavlink/mavros/tree/master/mavros#installation) 的简化版本。（官方安装指南请参见mavros功能包的github主页） 它们涵盖了 * ROS kinetic * 的版本。

### 二进制安装 (Debian/Ubuntu)

ROS 代码库有针对 Ubuntu x86、amd64 (x86\_64) 和 armhf (ARMv7) 的二进制安装包。 Kinetic also supports Debian Jessie amd64 and arm64 (ARMv8).

使用 ` apt-get` 进行安装:

    sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
    

然后通过运行 ` install_geographiclib_datasets.sh ` 脚本来安装 [ GeographicLib ](https://geographiclib.sourceforge.io/) 数据集:(译者注：注意这一步需要在命令前加sudo才会安装成功)

    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    ./install_geographiclib_datasets.sh
    

### Source Installation

This installation assumes you have a catkin workspace located at `~/catkin_ws` If you don't create one with:

```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
wstool init src
```

You will be using the ROS Python tools: *wstool* (for retrieving sources), *rosinstall*, and *catkin_tools* (building) for this installation. While they may have been installed during your installation of ROS you can also install them with:

```sh
sudo apt-get install python-catkin-tools python-rosinstall-generator -y
```

> **Tip** While the package can be built using **catkin_make** the preferred method is using **catkin_tools** as it is a more versatile and "friendly" build tool.

If this is your first time using wstool you will need to initialize your source space with:

```sh
$ wstool init ~/catkin_ws/src
```

Now you are ready to do the build

1. Install MAVLink: 
        # We use the Kinetic reference for all ROS distros as it's not distro-specific and up to date
        rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall

2. Install MAVROS from source using either released or latest version:
    
    - Released/stable ```rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall```
    - Latest source 
            sh
            rosinstall_generator --upstream-development mavros | tee -a /tmp/mavros.rosinstall
        
            sh
            # For fetching all the dependencies into your catkin_ws, 
            # just add '--deps' to the above scripts, E.g.:
            #   rosinstall_generator --upstream mavros --deps | tee -a /tmp/mavros.rosinstall

3. Create workspace & deps
    
        wstool merge -t src /tmp/mavros.rosinstall
        wstool update -t src -j4
        rosdep install --from-paths src --ignore-src -y
        

4. Install [GeographicLib](https://geographiclib.sourceforge.io/) datasets:
    
        ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
        

5. Build source
    
        catkin build
        

6. Make sure that you use setup.bash or setup.zsh from workspace.
    
        #Needed or rosrun can't find nodes from this workspace.
        source devel/setup.bash
        

In the case of error, there are addition installation and troubleshooting notes in the [mavros repo](https://github.com/mavlink/mavros/tree/master/mavros#installation).