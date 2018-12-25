# jMAVSim 进行 SITL 仿真

jMAVSim是一个简单的多旋翼/四旋翼仿真软件，它可以允许你在仿真环境中飞行运行着 PX4 的 *旋翼* 无人机。 它很容易设置，可以用来测试您的工具是否可以起飞、飞行、降落、并对各种故障条件 (例如 gps 故障) 做出适当的反应。

<strong>支持机型：</strong>

* 四旋翼

本问主要演示如何设置 jMAVSim 以连接到 SITL 版本的 PX4 。

> **Tip** jMAVSim 也可以用来进行 HITL 仿真 ([看这里](../simulation/hitl.md#using-jmavsim-quadrotor)).

## 仿真环境

软件在环仿真在主机上运行仿真系统的全部组件，使用软件来模拟真实飞控， 并通过当地网络实现与仿真软件的连接。 整套仿真方案设置如下：

{% mermaid %} graph LR; 仿真软件-->MAVLink; MAVLink-->SITL; {% endmermaid %}

## 运行 SITL

在确保 [仿真必备条件](../setup/dev_env.md) 已经安装在电脑系统上了之后直接运行命令：便捷的 make target 命令会完成 POSIX 平台的交叉编译并启动仿真。

```sh
make px4_sitl_default jmavsim
```

该命令最终将得到如下 PX4 控制台显示界面：

```sh
[init] shell id: 140735313310464
[init] task name: px4

______  __   __    ___ 
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

Ready to fly.


pxh>
```

此外，它还会唤起一个显示 [jMAVSim](https://github.com/PX4/jMAVSim) 模拟器的3D视图的窗口。

![jMAVSim 3d View](../../assets/simulation/jmavsim.jpg)

## 把飞机飞上天

系统将开始输出状态信息。 飞机完成位置锁定之后（控制台提示： *EKF commencing GPS fusion* 信息之后不久）你就可以开始飞行了。

在控制台输入以下命令进行起飞：

```sh
pxh> commander takeoff
```

你可以使用 *QGroundControl* 制定一个飞行计划，或者连接一个 [操纵杆](#joystick) 。

## 使用/配置选项

### 指定起飞位置

The default takeoff location in can be overridden using the environment variables: `PX4_HOME_LAT`, `PX4_HOME_LON`, and `PX4_HOME_ALT`.

For example, to set the latitude, longitude and altitude:

    export PX4_HOME_LAT=28.452386
    export PX4_HOME_LON=-13.867138
    export PX4_HOME_ALT=28.5
    make px4_sitl_default jmavsim
    

### 使用操纵杆 {#joystick}

Joystick and thumb-joystick support are supported through *QGroundControl* ([setup instructions here](../simulation/README.md#joystickgamepad-integration)).

### 模拟一个 Wifi 无人机

There is a special target to simulate a drone connected via Wifi on the local network:

```sh
make broadcast jmavsim
```

The simulator broadcasts its address on the local network as a real drone would do.

### 单独启动 JMAVSim 和 PX4

You can start JMAVSim and PX4 separately:

    ./Tools/jmavsim_run.sh
    make px4_sitl none
    

This allows a faster testing cycle (restarting jMAVSim takes significantly more time).

## 多飞行器仿真

JMAVSim can be used for multi-vehicle simulation: [Multi-Vehicle Sim with JMAVSim](../simulation/multi_vehicle_jmavsim.md).

## 扩展和定制

To extend or customize the simulation interface, edit the files in the **Tools/jMAVSim** folder. The code can be accessed through the[jMAVSim repository](https://github.com/px4/jMAVSim) on Github.

> **Info** The build system enforces the correct submodule to be checked out for all dependencies, including the simulator. It will not overwrite changes in files in the directory, however, when these changes are committed the submodule needs to be registered in the Firmware repo with the new commit hash. To do so, `git add Tools/jMAVSim` and commit the change. This will update the GIT hash of the simulator.

## Interfacing to ROS

The simulation can be [interfaced to ROS](../simulation/ros_interface.md) the same way as onboard a real vehicle.

## Important Files

* The startup script is in the [posix-configs/SITL/init](https://github.com/PX4/Firmware/tree/master/posix-configs/SITL/init) folder and named `rcS_SIM_AIRFRAME`, the default is `rcS_jmavsim_iris`.
* The root file system (the equivalent of `/` as seen by the) is located inside the build directory: `build/px4_sitl_default/src/firmware/posix/rootfs/`