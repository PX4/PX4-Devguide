# jMAVSim 进行 SITL 仿真

jMAVSim是一个简单的多旋翼/四旋翼仿真软件，它可以允许你在仿真环境中飞行运行着 PX4 的 *旋翼* 无人机。 它很容易设置，可以用来测试您的工具是否可以起飞、飞行、降落、并对各种故障条件 (例如 gps 故障) 做出适当的反应。

<strong>支持机型：</strong>

* 四旋翼

本问主要演示如何设置 jMAVSim 以连接到 SITL 版本的 PX4 。

> **Tip** jMAVSim 也可以用来进行 HITL 仿真 ([看这里](../simulation/hitl.md#using-jmavsim-quadrotor)).

## 仿真环境

软件在环仿真在主机上运行仿真系统的全部组件，使用软件来模拟真实飞控， 并通过当地网络实现与仿真软件的连接。 整套仿真方案设置如下：

{% mermaid %} graph LR; Simulator-->MAVLink; MAVLink-->SITL; {% endmermaid %}

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

![jMAVSim 3d 视图](../../assets/simulation/jmavsim.jpg)

## 把飞机飞上天

系统将开始输出状态信息。 飞机完成位置锁定之后（控制台提示： *EKF commencing GPS fusion* 信息之后不久）你就可以开始飞行了。

在控制台输入以下命令进行起飞：

```sh
pxh> commander takeoff
```

你可以使用 *QGroundControl* 制定一个飞行计划，或者连接一个 [操纵杆](#joystick) 。

## 使用/配置选项

### 指定起飞位置

通过设定环境变量可以覆盖默认的起飞地点设定： `PX4_HOME_LAT`, `PX4_HOME_LON`, 以及 `PX4_HOME_ALT` 。

举个例子，使用如下命令设定无人机起飞点的维度、经度和高度：

    export PX4_HOME_LAT=28.452386
    export PX4_HOME_LON=-13.867138
    export PX4_HOME_ALT=28.5
    make px4_sitl_default jmavsim
    

### Change Simulation Speed

The simulation speed can be increased or decreased with respect to realtime using the environment variable `PX4_SIM_SPEED_FACTOR`.

    export PX4_SIM_SPEED_FACTOR=2
    make px4_sitl_default jmavsim
    

For more information see: [Simulation > Run Simulation Faster than Realtime](../simulation/README.md#simulation_speed).

### Using a Joystick {#joystick}

Joystick and thumb-joystick support are supported through *QGroundControl* ([setup instructions here](../simulation/README.md#joystickgamepad-integration)).

### Simulating a Wifi Drone

There is a special target to simulate a drone connected via Wifi on the local network:

```sh
make broadcast jmavsim
```

The simulator broadcasts its address on the local network as a real drone would do.

### Start JMAVSim and PX4 Separately

You can start JMAVSim and PX4 separately:

    ./Tools/jmavsim_run.sh
    make px4_sitl none
    

This allows a faster testing cycle (restarting jMAVSim takes significantly more time).

## 多飞行器仿真

JMAVSim can be used for multi-vehicle simulation: [Multi-Vehicle Sim with JMAVSim](../simulation/multi_vehicle_jmavsim.md).

## 扩展和定制

To extend or customize the simulation interface, edit the files in the **Tools/jMAVSim** folder. The code can be accessed through the[jMAVSim repository](https://github.com/px4/jMAVSim) on Github.

> **Info** 编译系统会强制检查所有依赖项的子模块正确无误，其中就包括了模拟器。 但是，它不会直接覆盖你对目录中文件所做的更改， 当提交这些更改时你需要在固件 repo 中重新为子模块注册新的哈希值。 为此,，使用 `git add Tools/jMAVSim` 灵敏提交你的更改。 这将更新模拟器的 GIT 哈希值。

## 与 ROS 对接交互

The simulation can be [interfaced to ROS](../simulation/ros_interface.md) the same way as onboard a real vehicle.

## 重要的文件

* 启动脚本位于 [posix-configs/SITL/init](https://github.com/PX4/Firmware/tree/master/posix-configs/SITL/init) 文件夹下，以 `rcS_SIM_AIRFRAME` 的方式进行命名，默认值是 `rcS_jmavsim_iris` 。
* 根文件系统 (相当于 `/`) 位于生成目录内: `build/px4_sitl_default/src/firmware/posix/rootfs/` 。