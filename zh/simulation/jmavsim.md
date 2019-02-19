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
    

### 更改仿真的时间流速

可以使用环境变量 `PX4_SIM_SPEED_FACTOR` 提高或者降低仿真环境的时间流速相对于实际时间流速的大小。

    export PX4_SIM_SPEED_FACTOR=2
    make px4_sitl_default jmavsim
    

更多相关信息请参考：[Simulation > Run Simulation Faster than Realtime](../simulation/README.md#simulation_speed).

### 使用游戏手柄 {#joystick}

通过 *QGroundControl* 可引入游戏手柄或者拇指操纵杆（[如何进行设置看这里](../simulation/README.md#joystickgamepad-integration)）。

### 模拟一个 Wifi 无人机

有一个特殊的平台可以模拟一个通过 Wifi 进行连接的无人机。

```sh
make broadcast jmavsim
```

模拟器会跟真实的该类无人机一样在当地网络中广播自己的位置信息等。

### 分别启动 JMAVSim 和 PX4

你可以单独启动 JMAVSim 和 PX4:

    ./Tools/jmavsim_run.sh
    make px4_sitl none
    

此举可以缩短测试循环时间（重启 jMAVSim 需要耗费非常多的时间）。

## 多飞行器仿真

JMAVSim 也可用来进行多飞行器仿真： [Multi-Vehicle Sim with JMAVSim](../simulation/multi_vehicle_jmavsim.md).

## 扩展和定制

如果想扩展或者定制仿真接口，你可以编辑 **Tools/jMAVSim** 文件夹下的文件： 源代码可以从 Github 上的 [jMAVSim 软件仓库](https://github.com/px4/jMAVSim) 获取。

> **Info** 编译系统会强制检查所有依赖项的子模块正确无误，其中就包括了模拟器。 但是，它不会直接覆盖你对目录中文件所做的更改， 当提交这些更改时你需要在固件 repo 中重新为子模块注册新的哈希值。 为此,，使用 `git add Tools/jMAVSim` 灵敏提交你的更改。 这将更新模拟器的 GIT 哈希值。

## 与 ROS 对接交互

在仿真中可以使用跟真实飞机一样的方式实现 [与 ROS 的对接交互](../simulation/ros_interface.md) 。

## 重要的文件

* 启动脚本位于 [posix-configs/SITL/init](https://github.com/PX4/Firmware/tree/master/posix-configs/SITL/init) 文件夹下，以 `rcS_SIM_AIRFRAME` 的方式进行命名，默认值是 `rcS_jmavsim_iris` 。
* The simulated root file system ("`/`" directory) is created inside the build directory here: `build/px4_sitl_default/tmp/rootfs`.