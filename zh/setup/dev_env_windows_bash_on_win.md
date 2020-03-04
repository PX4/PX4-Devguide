# Bash on Windows 工具链

> **注意：** [Windows Cygwin 工具链](../setup/dev_env_windows_cygwin.md) 是Windows平台唯一获得官方支持的开发环境。

Windows 用户还可以选择在 [Bash on Windows](https://github.com/Microsoft/BashOnWindows) 中安装经过 *少许修改* 的基于Ubuntu Linux的PX4 开发环境 ，该开发环境可用于：

* 编译针对 NuttX/Pixhawk 平台的固件。
* 使用 JMAVSim 进行PX4仿真 (需要搭配一个基于Windows的 X-Windows 应用来显示仿真UI界面)。

> **注意：** 本特性仅可在Windows 10上实现， 它本质上仍是在虚拟机中运行工具链, 与其他解决方案相比运行相对缓慢。

### 设置开发环境

The easiest way to setup the environment is to use the **<a href="https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/windows_bash_nuttx.sh" target="_blank" download>windows_bash_nuttx.sh</a>** script (details for script are [given below](#build_script_details)).

要设置开发环境, 请执行以下操作:

1. 在Windows 10上启用、安装 [Bash on Windows](https://github.com/Microsoft/BashOnWindows)。
2. 打开 bash shell 命令行界面。
3. Download the **windows_bash_nuttx.sh**:  
    `wget https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/windows_bash_nuttx.sh`
4. 使用如下命令运行安装脚本 (根据需要确认任何提示)： 
        sh
        bash windows_bash_nuttx.sh

### 编译固件

固件编译流程（以编译 px4_fmu-v4 的固件为例）：

1. 在 bash shell 命令行界面输入如下指令：
    
        cd ~/src/Firmware
        make px4_fmu-v4_default
        
    
    成功完成编译后可以在 `Firmware/build/px4_fmu-v4_default/px4_fmu-v4_default.px4` 文件夹下找到编译好的固件。
    
    > **Note** The `make` commands to build firmware for other boards can be found in [Building the Code](../setup/building_px4.md#nuttx)

2. 在 Windows 平台上无法直接在 bash shell 中使用 `upload` 命令完成固件的烧写，你可以使用 *QGroundControl* 或者 *Mission Planner* 烧写自定义的固件。

### 仿真模拟 （JMAVSim）

Bash on Windows 并不包括任何UI库的支持。 为了显示 jMAVSim 的UI界面，在进行仿真之前你需要在 Windows 平台上安装 X-Window 图形用户接口应用，比如： [XMing](https://sourceforge.net/projects/xming/)。

JMAVSim 运行流程：

1. 在 Windows 平台安装并启动 [XMing](https://sourceforge.net/projects/xming/)。
2. 在 bash shell 命令行界面输入如下指令： 
        sh
        export DISPLAY=:0 > 
    
    **提示：** 将上一行命令加入 Ubuntu 的 **.bashrc** 文件末尾可避免在新的 bash 会话中重复输入该命令。
3. 在 bash shell 界面中启动 px4 和 jmavsim：
    
    ```sh
    make px4_sitl jmavsim
    ```
    
    JMAVSim 的UI界面会显示在 XMing 程序中，如下所示：
    
    ![jMAVSimOnWindows](../../assets/simulation/JMAVSim_on_Windows.PNG)

> **注意！** Gazebo 也可以以类似方式在 Ubuntu Bash for Windows 中运行，但运行速度太慢以至于没有实用价值。 如要尝试运行，请遵循 [ROS kinetic install guide](http://wiki.ros.org/kinetic/Installation/Ubuntu) 的指示然后在 Bash shell 界面中使用如下命令运行Gazebo： 
> 
>     sh
>       export DISPLAY=:0
>       export GAZEBO_IP=127.0.0.1
>       make px4_sitl gazebo

### 开发环境安装脚本详情 {#build_script_details}

The [windows_bash_nuttx.sh](https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/windows_bash_nuttx.sh) build script modifies the Ubuntu build instructions to remove Ubuntu-specific and UI-dependent components, including the *Qt Creator* IDE and the simulators.

In addition, it uses a [64 bit arm-none-eabi compiler](https://github.com/SolinGuo/arm-none-eabi-bash-on-win10-.git) since BashOnWindows doesn't run 32 bit ELF programs (and the default compiler from `https://launchpad.net/gcc-arm-embedded` is 32 bit).

手动将此编译器添加到您的环境中请执行以下操作:

1. 下载编译器: 
        sh
        wget https://github.com/SolinGuo/arm-none-eabi-bash-on-win10-/raw/master/gcc-arm-none-eabi-5_4-2017q2-20170512-linux.tar.bz2

2. Bash On Windows 控制台中使用命令行进行解压： 
        sh
        tar -xvf gcc-arm-none-eabi-5_4-2017q2-20170512-linux.tar.bz2 该命令会将 arm gcc cross-compiler 解压至： ```gcc-arm-none-eabi-5_4-2017q2/bin```

3. 将下面这行命令添加到环境中（将该行添加到 bash 配置文件中完成永久性更改） ```export PATH=$HOME/gcc-arm-none-eabi-5_4-2017q2/bin:$PATH```