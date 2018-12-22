# Bash on Windows 工具链

> **注意：** [Windows Cygwin 工具链](../setup/dev_env_windows_cygwin.md) 是Windows平台唯一获得官方支持的开发环境。

Windows 用户还可以选择在 [Bash on Windows](https://github.com/Microsoft/BashOnWindows) 中安装经过 *少许修改* 的基于Ubuntu Linux的PX4 开发环境 ，该开发环境可用于：

* 编译针对 NuttX/Pixhawk 平台的固件。 
* 使用 JMAVSim 进行PX4仿真 (需要搭配一个基于Windows的 X-Windows 应用来显示仿真UI界面)。

> **注意** 本特性仅可在Windows 10上实现， 它本质上仍是在虚拟机中运行工具链, 与其他解决方案相比运行相对缓慢。

### Setup Environment

The easiest way to setup the environment is to use the **<a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/windows_bash_nuttx.sh" target="_blank" download>windows_bash_nuttx.sh</a>** script (details for script are [given below](#build_script_details)).

To setup the development environment:

1. Install [Bash on Windows](https://github.com/Microsoft/BashOnWindows).
2. Open the bash shell. 
3. Download the **windows_bash_nuttx.sh**: ```wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/windows_bash_nuttx.sh```
4. Run the script using the command below (acknowledging any prompts as required): 
        sh
        source windows_bash_nuttx.sh

### Build Firmware

To build the firmware (i.e. for px4_fmu-v4):

1. Enter the following commands in the bash shell:
    
        cd ~/src/Firmware
        make px4_fmu-v4_default
        
    
    On successful completion you'll find the firmware here: `Firmware/build/px4_fmu-v4_default/px4_fmu-v4_default.px4`
    
    > **Note** The `make` commands to build firmware for other boards can be found in [Building the Code](../setup/building_px4.md#nuttx--pixhawk-based-boards)

2. You can flash the custom firmware on Windows using *QGroundControl* or *Mission Planner* (it is not possible to directly flash the firmware from within the bash shell using the `upload` command).

### Simulation (JMAVSim)

Bash on Windows does not include support for UI libraries. In order to display the jMAVSim UI you will first need to install an X-Window application like [XMing](https://sourceforge.net/projects/xming/) into Windows.

To run JMAVSim:

1. Install and start [XMing](https://sourceforge.net/projects/xming/) on Windows.
2. Enter the following command in the bash shell: 
        sh
        export DISPLAY=:0 > 
    
    **Tip** Add this line to the Ubuntu **.bashrc** file if you don't want to enter it every session.
3. Start PX4 and jMAVSim in the bash shell:
    
    ```sh
    make px4_sitl jmavsim
    ```
    
    The JMAVSim UI is then displayed in XMing as shown below:
    
    ![jMAVSimOnWindows](../../assets/simulation/JMAVSim_on_Windows.PNG)

> **Caution** Gazebo can similarly be run within Ubuntu Bash for Windows, but too slow to be useful. To try this, follow the [ROS kinetic install guide](http://wiki.ros.org/kinetic/Installation/Ubuntu) and run Gazebo in the Bash shell as shown: 
> 
>     sh
>       export DISPLAY=:0
>       export GAZEBO_IP=127.0.0.1
>       make px4_sitl gazebo

### Build Script Details {#build_script_details}

The [windows_bash_nuttx.sh](https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/windows_bash_nuttx.sh) build script modifies the Ubuntu build instructions to remove Ubuntu-specific and UI-dependent components, including the *Qt Creator* IDE and the simulators.

In addition, it uses a [64 bit arm-none-eabi compiler](https://github.com/SolinGuo/arm-none-eabi-bash-on-win10-.git) since BashOnWindows doesn't run 32 bit ELF programs (and the default compiler from `https://launchpad.net/gcc-arm-embedded` is 32 bit).

To add this compiler to your environment manually:

1. Download the compiler: 
        sh
        wget https://github.com/SolinGuo/arm-none-eabi-bash-on-win10-/raw/master/gcc-arm-none-eabi-5_4-2017q2-20170512-linux.tar.bz2

2. Unpack it using this command line in the Bash On Windows console: 
        sh
        tar -xvf gcc-arm-none-eabi-5_4-2017q2-20170512-linux.tar.bz2 This will unpack the arm gcc cross-compiler to: ```gcc-arm-none-eabi-5_4-2017q2/bin```

3. Add the to the environment (add the line to your bash profile to make the change permanent) ```export PATH=$HOME/gcc-arm-none-eabi-5_4-2017q2/bin:\$PATH```