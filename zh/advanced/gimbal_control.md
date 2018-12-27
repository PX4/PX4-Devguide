# 云台控制安装设置

If you want to control a gimbal with a camera (or any other payload) attached to the vehicle, you need to configure how you want to control it and how PX4 can command it. 本页内容就是讲解如何去安装，以及配置。

PX4 contains a generic mount/gimbal control driver with different input and output methods. The input defines how you control the gimbal: via RC or via MAVLink commands (for example in missions or surveys). The output defines how the gimbal is connected: some support MAVLink commands, others use PWM (described as AUX output in the following). Any input method can be selected to drive any output. 两种方式都需要去通过参数配置。

## 参数

[These parameters](../advanced/parameter_reference.md#mount) are used to setup the mount driver. The most important ones are the input (`MNT_MODE_IN`) and the output (`MNT_MODE_OUT`) mode. 默认情况下，输入是没有被使能的，并且这个驱动没有运行。 After selecting the input mode, reboot the vehicle so that the mount driver starts.

If the input mode is set to `AUTO`, the mode will automatically be switched based on the latest input. To switch from MAVLink to RC, a large stick motion is required.

## 辅助输出

If the output mode is set to `AUX`, a mixer file is required to define the mapping for the output pins and the [mount mixer](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/mixers/mount.aux.mix) is automatically selected (overriding any aux mixer provided by the airframe configuration).

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

台风H480的模型带有一个预先配置的模拟云台。 若要运行它，请使用：

    make px4_sitl gazebo_typhoon_h480
    

To just test the mount driver on other models or simulators, make sure the driver runs, using `vmount start`, then configure its parameters.

## 测试

驱动程序提供了一个简单的测试指令。他需要首先使用“vmount stop”指令来停止。 接下来描述了在SITL中的测试方式，但是这些指令也可以在真实的设备中运行。

使用下面这条指令开始仿真：（没有参数需要被修改）

    make px4_sitl gazebo_typhoon_h480
    

Make sure it's armed, eg. with `commander takeoff`, then use for example

    vmount test yaw 30
    

to control the gimbal. Note that the simulated gimbal stabilizes itself, so if you send mavlink commands, set the `stabilize` flags to false.

![Gazebo Gimbal Simulation](../../assets/gazebo/gimbal-simulation.png)