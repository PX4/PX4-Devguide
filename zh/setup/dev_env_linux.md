---
translated_page: https://github.com/PX4/Devguide/blob/master/en/setup/dev_env_linux.md
translated_sha: 23bf0248facb1ab7d0dd58003c3234a95f479931
---

# Linux开发环境
我们使用Debian / Ubuntu LTS 作为Linux的标准支持版本，但是也支持[Cent OS 和 Arch Linux的发行版本](../setup/dev_env_linux_boutique.md)。

## 权限设置


> 警告：永远不要使用`sudo`来修复权限问题，否则会带来更多的权限问题，需要重装系统来解决。


把用户添加到用户组　"dialout":

<div class="host-code"></div>

```sh
sudo usermod -a -G dialout $USER
```

然后注销后，重新登录，因为重新登录后所做的改变才会有效。

## 安装

更新包列表，安装下面编译PX4的依赖包。PX4主要支持的系列：

* NuttX based hardware: [Pixhawk](../flight_controller/pixhawk.md), [Pixfalcon](../flight_controller/pixfalcon.md),
  [Pixracer](../flight_controller/pixracer.md), [Crazyflie](../flight_controller/crazyflie2.md),
  [Intel® Aero Ready to Fly Drone](../flight_controller/intel_aero.md)
* [Qualcomm Snapdragon Flight hardware](../flight_controller/snapdragon_flight.md)
* Linux-based hardware: [Raspberry Pi 2/3](../flight_controller/raspberry_pi.md)、, Parrot Bebop
* Host simulation: [jMAVSim SITL](../simulation/sitl.md) and [Gazebo SITL](../simulation/gazebo.md)

> 提示：安装[Ninja Build System](../setup/dev_env_linux_boutique.md#ninja-build-system)可以比make更快进行编译。如果安装了它就会自动选择使用它进行编译。


<div class="host-code"></div>

```sh
sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
sudo apt-get update
# 必备软件
sudo apt-get install python-argparse git-core wget zip \
    python-empy qtcreator cmake build-essential genromfs -y
# 仿真工具
sudo add-apt-repository ppa:openjdk-r/ppa
sudo apt-get update
sudo apt-get install openjdk-8-jre
sudo apt-get install ant protobuf-compiler libeigen3-dev libopencv-dev openjdk-8-jdk openjdk-8-jre clang-3.5 lldb-3.5 -y
```

### 基于NuttX的硬件

Ubuntu配备了一系列代理管理，这会严重干扰任何机器人相关的串口（或usb串口），卸载掉它也不会有什么影响:

<div class="host-code"></div>

```sh
sudo apt-get remove modemmanager
```

更新包列表和安装下面的依赖包。务必安装指定的版本的包

<div class="host-code"></div>

```sh
sudo apt-get install python-serial openocd \
    flex bison libncurses5-dev autoconf texinfo build-essential \
    libftdi-dev libtool zlib1g-dev \
    python-empy  -y
```

在添加arm-none-eabi工具链之前，请确保删除残余。

```sh
sudo apt-get remove gcc-arm-none-eabi gdb-arm-none-eabi binutils-arm-none-eabi gcc-arm-embedded
sudo add-apt-repository --remove ppa:team-gcc-arm-embedded/ppa
```

如果`gcc-arm-none-eabi`版本导致PX4/Firmware编译错误，请参考[the bare metal installation instructions](../setup/dev_env_linux_boutique.md#toolchain-installation)手动安装4.9或者5.4版本的arm-none-eabi工具链。

### 骁龙

#### 工具链安装

```sh
sudo apt-get install android-tools-adb android-tools-fastboot fakechroot fakeroot unzip xz-utils wget python python-empy -y
```

```sh
git clone https://github.com/ATLFlight/cross_toolchain.git
```


从QDN上获得Hexagon SDK 3.0: [https://developer.qualcomm.com/download/hexagon/hexagon-sdk-v3-linux.bin](https://developer.qualcomm.com/download/hexagon/hexagon-sdk-v3-linux.bin)

需要登录到QDN。如果没有账户，你需要注册一个。

现在需要移动这些为交叉编译工具链下载的文件：

```sh
mv ~/Downloads/hexagon-sdk-v3-linux.bin cross_toolchain/downloads
```

安装工具链和SDK：

```sh
cd cross_toolchain
./installv3.sh
cd ..
```

按照说明配置开发环境。如果你接受默认安装选项，则可以在之后随时重新运行下面的命令来配置开发环境。它只会安装缺少的组件。

执行后工具和SDK将被安装到"$HOME/Qualcomm/..."。 把这些添加到 ~/.bashrc:

```sh
export HEXAGON_SDK_ROOT="${HOME}/Qualcomm/Hexagon_SDK/3.0"
export HEXAGON_TOOLS_ROOT="${HOME}/Qualcomm/HEXAGON_Tools/7.2.12/Tools"
export PATH="${HEXAGON_SDK_ROOT}/gcc-linaro-4.9-2014.11-x86_64_arm-linux-gnueabihf_linux/bin:$PATH"
```

载入新的参数：

```sh
source ~/.bashrc
```

#### Sysroot安装

根系统目录提供了交叉编译骁龙飞控所必要的链接库和头文件。

qrlSDK sysroot提供了摄像头、GPU等需要的头文件和链接库。

下载 [Flight\_3.1.1\_qrlSDK.zip](http://support.intrinsyc.com/attachments/download/690/Flight_3.1.1_qrlSDK.zip) 并保存到`cross_toolchain/download/`。

```sh
cd cross_toolchain
unset HEXAGON_ARM_SYSROOT
./qrlinux_sysroot.sh
```
把这些添加到 ~/.bashrc：

```sh
export HEXAGON_ARM_SYSROOT=${HOME}/Qualcomm/qrlinux_v3.1.1_sysroot
```

载入新的配置：

```sh
source ~/.bashrc
```

根系统更多选项参见[这里](https://github.com/ATLFlight/cross_toolchain/blob/sdk3/README.md#sysroot-installation)

#### 升级ADSP固件

在构建，烧写以及运行代码之前，还需要升级[ADSP固件](../flight_controller/snapdragon_flight_advanced.md#updating-the-adsp-firmware)。

#### 参考

[GettingStarted](https://github.com/ATLFlight/ATLFlightDocs/blob/master/GettingStarted.md)是另外一个工具链安装向导。[HelloWorld](https://github.com/ATLFlight/HelloWorld)和[DSPAL tests](https://github.com/ATLFlight/dspal/tree/master/test/dspal_tester)可以用来验证工具链安装和DSP镜像。

DSP的信息可以通过mini-dm查看。

<div class="host-code"></div>

```sh
$HOME/Qualcomm/Hexagon_SDK/2.0/tools/mini-dm/Linux_Debug/mini-dm
```
> Note: 在MAC上也可以使用[nano-dm](https://github.com/kevinmehall/nano-dm).

### 树莓派

树莓派开发者应该从下面地址下载树莓派Linux工具链。安装脚本会自动安装交叉编译工具链。如果想要用原生树莓派工具链在树莓派上直接编译，参见[这里](../flight_controller/raspberry_pi.md#native-builds-optional)。

<div class="host-code"></div>

```sh
git clone https://github.com/pixhawk/rpi_toolchain.git
cd rpi_toolchain
./install_cross.sh
```

在工具链安装过程中需要输入密码。

如果不想把工具链安装在默认位置```/opt/rpi_toolchain```，可以执行``` ./install_cross.sh <PATH>```向安装脚本传入其它地址。安装脚本会自动配置需要的环境变量。

最后，执行该命令更新环境变量：
```
source ~/.profile
```

### Parrot Bebop

Parrot Bebop的开发者应当安装树莓派的Linux工具链。请跟随这里的介绍[树莓派2/3自动驾驶仪](../flight_controller/raspberry_pi.md).

接下来安装ADB。

``sh      
sudo apt-get install android-tools-adb -y ``

## 完成

继续，进行[第一次代码编译](../setup/building_px4.md)!
