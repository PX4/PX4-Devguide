# 云台控制安装设置

如果你想要去控制一个装在飞机上的带相机的云台（或者是其他任何的载荷），并且想通过PX4去控制它，你需要配置一些参数， 本页内容就是讲解如何去安装，以及配置。

PX4包含了一个通用的云台/挂载设备的控制驱动器，它含有多种输入输出方式。 输入方式的意思是你想通过什么方式去控制云台：可以用过RC（遥控器）或者通过MAVLINK命令（一般是地面站里面的任务模式或者是航域模式）。 输出的定义是怎么去连接云台：一些云台支持MAVLINK命令，还有一些云台用PWM方式（下面描述的是AUX辅助输出方式）。 任何的输入可以被选择去驱动任何的输出。 两种方式都需要去通过参数配置。

## 参数

这些[参数](../advanced/parameter_reference.md#mount) 被用作去安装一个挂载设备驱动。 最重要的 是输入 (` MNT_MODE_IN `) 和输出 (` MNT_MODE_OUT `) 模式。 默认情况下，输入是没有被使能的，并且这个驱动没有运行。 在选择了输入模式后，重启飞机以便挂在设备驱动运行起来。

如果输入模式设置为 ` AUTO `, 则模式将根据最新输入进行自动切换 。 去切换MAVLINK到RC，需要一个较大的杆量动作。

## 辅助输出

如果输出模式设置为`AUX`，需要定义混控器文件去重新映射输出引脚和自动选择的[mount mixer](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/mixers/mount.aux.mix)（覆盖机型配置文件提供的辅助通道混控器）

输出分配如下所示:

- **AUX1**: Pitch
- **AUX2**: Roll
- **AUX3**: Yaw
- **AUX4**: Shutter/retract

### 自定义混控器配置

> **Note** Read [Mixing and Actuators](../concept/mixing.md) for an explanation of how mixers work and the format of the mixer file.

The outputs can be customized by [creating a mixer file](../concept/system_startup.md#starting-a-custom-mixer) on the SD card with name `etc/mixers/mount.aux.mix`.

下面列举的是一针对挂载设备的基本的混控器配置：

    # roll
    M: 1
    O:      10000  10000      0 -10000  10000
    S: 2 0  10000  10000      0 -10000  10000
    
    # pitch
    M: 1
    O:      10000  10000      0 -10000  10000
    S: 2 1  10000  10000      0 -10000  10000
    
    # yaw
    M: 1
    O:      10000  10000      0 -10000  10000
    S: 2 2  10000  10000      0 -10000  10000
    

## 软件在环仿真

台风H480的模型带有一个预先配置的模拟云台。 要运行它的话，使用下面的make语句：

    make posix gazebo_typhoon_h480
    

要在其他模型或模拟器上测试挂载驱动程序，请确保驱动程序运行，使用vmount start，然后配置其参数。

## 测试

驱动程序提供了一个简单的测试指令。他需要首先使用“vmount stop”指令来停止。 接下来描述了在SITL中的测试方式，但是这些指令也可以在真实的设备中运行。

使用下面这条指令开始仿真：（没有参数需要被修改）

    make posix gazebo_typhoon_h480
    

Make sure it's armed, eg. with `commander takeoff`, then use for example

    vmount test yaw 30
    

to control the gimbal. Note that the simulated gimbal stabilizes itself, so if you send mavlink commands, set the `stabilize` flags to false.

![Gazebo Gimbal Simulation](../../assets/gazebo/gimbal-simulation.png)