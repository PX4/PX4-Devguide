# Arch Linux 上开发环境的搭建

固件仓库里已经提供了一个脚本 [Tools/setup/arch.sh](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/Tools/setup/arch.sh) 方便你快速地在你的平台上搭建 PX4 的开发环境。

该脚本默认安装所有必需的工具，用于编译基于NuttX的PX4源码（不带RTPS），以及运行基于 *jMAVsim* 的仿真器。 你也可以安装额外的*Gazebo*仿真器通过在命令行中指定一个参数： `--gazebo`。

![Arch上使用Gazebo](../../assets/gazebo/arch-gazebo.png)

> **Note** 所有的指令已经在[Manjaro](https://manjaro.org/)（Arch的衍生）上做过测试，因为他比在Arch Linux上更容易搭建。

要获取并运行这个脚本，有下面几种办法：

- [下载PX4的源码，](../setup/building_px4.md) 然后运行这个目录下的脚本： 
        git clone https://github.com/PX4/Firmware.git
        bash Firmware/Tools/setup/arch.sh

- 只下载你所需的脚本并运行他们： 
        sh
        wget https://raw.githubusercontent.com/PX4/Firmware/master/Tools/setup/arch.sh
        wget https://raw.githubusercontent.com/PX4/Firmware/master/Tools/setup/requirements.txt
        bash arch.sh

该脚本可以加入这些可选的参数：

- `--gazebo`：添加此参数从 [AUR](https://aur.archlinux.org/packages/gazebo/) 安装 Gazebo 仿真器。 > **Note** Gazebo 是通过获取源码后编译得到的， 这个过程需要花费一些时间并且需要添加 `sudo` 并多次输入密码（对于依赖项）。
- `--no-nuttx`: 不要安装 NuttX/Pixhawk 的工具链 (比如你只想使用仿真器的功能)。
- `--no-sim-tools`：不要安装 jMAVSim/Gazebo 仿真器（例如你只想使用或者开发调试 Pixhawk/NuttX）