# 仿真

在仿真机中模拟器允许 px4 飞行代码来控制计算机建模工具。 您可以与此工具进行交互，就像您可以使用 *QGroundControl*、非机载 api 或无线电控制器/游戏板与真正的车辆进行交互一样。

> **Tip**仿真是一种快速、简单、最重要的方法，*safe* 的方法来测试对 px4 代码的更改，然后再尝试在现实世界中飞行。 当你还没有飞行器可以试验的时候，使用 px4 来模拟飞行的就是一种好方法。

Px4 支持 *软件在环（SITL）* 仿真，其中飞行堆栈在计算机上运行（同一台计算机或同一网络上的另一台计算机），也支持 *硬件在环（HITL）*仿真，即使用真实飞行电路板来运行仿真。

下一节将提供有关可用仿真器以及如何配置仿真仿真器的信息。 其他部分提供了有关仿真器如何工作的普通信息, 并且不需要 *use* 模拟器。

## 支持的仿真器

以下仿真器与 px4 一起工作，用于 HITL 和/或 SITL 仿真。

| 仿真器                               | 描述 |
| --------------------------------- | -- |
| [Gazebo](../simulation/gazebo.md) |    |

**强烈建议使用此仿真器。**

它具有功能强大的 3D 仿真环境, 特别适用于测试对象避障和计算机视觉。 它还可用于 [多工具仿真](../simulation/multi-vehicle-simulation.md)，通常用于 [ROS](../simulation/ros_interface.md)，这是一种用于自动控制的工具集。 

**支持机型：**四旋翼 ([Iris](../airframes/airframe_reference.md#copter_quadrotor_wide_3dr_iris_quadrotor) 和 [Solo](../airframes/airframe_reference.md#copter_quadrotor_x_3dr_solo))，六旋翼 (Typhoon h480)，[通用四旋翼 delta VTOL 无人机](../airframes/airframe_reference.md#vtol_standard_vtol_generic_quad_delta_vtol)，尾翼，飞机，探测车，潜艇（即将推出！） 

[jMAVSim](../simulation/jmavsim.md) | 一个简单的多旋翼仿真器，允许在仿真机中使用 *copter* 型无人机。

它易设置，可以用来测试您的工具是否可以起飞、飞行、降落、并对各种故障条件 (例如 gps 故障) 做出适当的反应。 它也可用于 多机仿真 </0 >。</p> 

**支持机型： **四旋翼

[AirSim](../simulation/airsim.md) | 提供物理和视觉逼真模拟的跨平台仿真器。 这个模拟器需要大量的资源，需要一台比这里描述的其他仿真器更强大的计算机。

**支持机型: </0 >Iris （多转子模型和 x 配置中 px4 quadrotor 的配置）。</p> 

[XPlane](../simulation/hitl.md)（仅硬件在环）| 一个全面而强大的固定翼飞行仿真器，提供非常逼真的飞行模型。  


**支持机型： **四旋翼

有关如何设置和使用仿真器的说明，请参见上面链接的主题。

* * *

本主题的其余部分是对仿真基础结构如何工作的 "有点笼统" 的描述。 它不需要 *use* 仿真器。

## 仿真器 MAVLink API

所有模拟器都使用 Simulator MAVLink API 与 PX4 进行通信。 该 API 定义了一组 MAVLink 消息，这些消息将仿真机的传感器数据提供给 PX4，并从将应用于仿真机的飞行代码返回电机和执行器值。 下面图表描述了消息。

![仿真器 MAVLink API](../../assets/simulation/px4_simulator_messages.png)

> **注意** PX4 的 SITL 版本使用[仿真器 mavlink.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/simulator/simulator_mavlink.cpp)来处理这些消息，而在HITL模式下的硬件构建使用[mavlink receiver.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.cpp)。 模拟器中的传感器数据将写入 PX4 uORB 主题。 所有电机/执行器都被卡停，但内部软件可以完全正常运行。

下面介绍了这些消息 （有关特定详细信息, 请参阅链接）。

| 消息                                                                                                             | 方向        | 描述                                                                              |
| -------------------------------------------------------------------------------------------------------------- | --------- | ------------------------------------------------------------------------------- |
| [MAV_MODE:MAV_MODE_FLAG_HIL_ENABLED](https://mavlink.io/en/messages/common.html#MAV_MODE_FLAG_HIL_ENABLED) | 不可用       | 使用模拟时的模式标志。 所有电机/执行器都被卡停，但内部软件可以完全正常运行。                                         |
| [HIL_ACTUATOR_CONTROLS](https://mavlink.io/en/messages/common.html#HIL_ACTUATOR_CONTROLS)                    | PX4 至 Sim | PX4 控制输出 （电机、执行器）。                                                              |
| [HIL_SENSOR](https://mavlink.io/en/messages/common.html#HIL_SENSOR)                                            | Sim 至 PX4 | 在 NED 体框架中以 SI 单位模拟 IMU 读数。                                                     |
| [HIL_GPS](https://mavlink.io/en/messages/common.html#HIL_GPS)                                                  | Sim 至 PX4 | 模拟的 GPS RAW 传感器值。                                                               |
| [HIL_OPTICAL_FLOW](https://mavlink.io/en/messages/common.html#HIL_OPTICAL_FLOW)                              | Sim 至 PX4 | 来自流量传感器的模拟光流 （例如 PX4FLOW 或光学鼠标传感器）。                                             |
| [HIL_STATE_QUATERNION](https://mavlink.io/en/messages/common.html#HIL_STATE_QUATERNION)                      | Sim 至 PX4 | 包含实际的“仿真”无人机位置、姿态、速度等。 这可以记录并与 px4 的分析和调试估计进行比较 （例如，检查估计器在噪声 （仿真） 传感器输入中的工作情况）。 |
| [HIL_RC_INPUTS_RAW](https://mavlink.io/en/messages/common.html#HIL_RC_INPUTS_RAW)                            | Sim 至 PX4 | 收到 RC 通道的 RAW 值。                                                                |

## 默认 PX4 MAVLink UDP 端口

默认情况下，PX4 使用通常建立的 UDP 端口与地面控制站（例如，*QGroundControl*），外部 API（例如Dronecode SDK，MAVROS）和模拟器 API（例如 Gazebo）进行 MAVLink 通信。 这些端口是：

* 端口** 14540 **用于与外接 APIs 通信。 期望外接 APIs 监听此端口上的连接。
* 端口** 14550 **用于与地面控制站通信。 期望 GCS 将侦听此端口上的连接。 *QGroundControl*默认侦听此端口。
* 端口** 14560 **用于与外接 APIs 通信。 PX4 侦听此端口，仿真器应通过向该端口广播数据来启动通信。

> **注意**GCS 和外置 API 的端口设置在配置文件中，而仿真器广播端口在模拟 MAVlink 模块中硬编码．

## SITL 仿真环境

下面显示了适用于任何受支持仿真器的典型 SITL 仿真环境。 系统的不同部分通过 UDP 连接，并且可以在同一台计算机上运行，也可以在同一网络上的另一台计算机上运行。

* PX4 使用一个特定仿真的模块来侦听 UDP 端口 14560。 模拟器连接到此端口，然后使用上面描述的 [Simulator mavlink API](#simulator-mavlink-api) 交换信息。 SITL 和模拟器上的 PX4 可以在同一台计算机上运行，也可以在同一网络上运行不同的计算机。
* PX4 使用普通的 MAVLink 模块连接到 GroundStations（侦听端口 14550）和外部开发人员 API，如Dronecode SDK 或 ROS（侦听端口 14540）。
* 串行连接用于通过 *QGroundControl* 连接 Joystick/Gamepad 硬件。

![PX4 SITL 概述](../../assets/simulation/px4_sitl_overview.png)

如果使用正常的生成系统 SITL `make` 配置目标 （请参阅下一节），则 SITL 和模拟器都将在同一台计算机上启动，并自动配置上述端口。 您可以配置其他 MAVLink UDP 连接，并以其他方式修改生成配置和初始化文件中的模拟环境。

### 启动/构建 SITL 模拟

构建系统使在 SITL 上构建和启动 PX4、启动模拟器并连接它们变得非常容易。 语法 （简化）如下所示：

    make px4_sitl simulator[_vehicle-model]
    

其中 `simulator` 是 `gazebo`、`jmavsim` 或其他一些模拟器，该设备模型是该模拟器支持的特殊的无人机类型 （[jMAVSim](../simulation/jmavsim.md) 仅支持多路光台，而 [Gazebo](../simulation/gazebo.md) 支持许多不同类型）。

下面显示了许多示例，每个模拟器的各个页面中还有更多示例：

```sh
# 启动固定翼机型的 Gazebo
make px4_sitl gazebo_plane

# 启动有光流的Iris机型的 Gazebo
make px4_sitl gazebo_iris_opt_flow

# 启动Iris（默认机型）的 JMavSim
make px4_sitl jmavsim
```

可以通过环境变量进一步仿真机：

* `PX4_ESTIMATOR`：此变量配置要使用的估算器。 可能的选项有：`ekf2` （默认）、`lpe`、`inav`。 在运行模拟之前，可以通过 `export PX4_ESTIMATOR=lpe` 进行设置。

这里描述的语法是简化的，您可以通过 *make* 配置许多其他选项，例如，设置要连接到 IDE 或调试器的选项。 有关详细信息，请参阅： [Building 代码 > PX4 使生成 Targets](../setup/building_px4.md#make_targets)。

### 以比实际时间更快的流速运行仿真 {#simulation_speed}

使用 jMAVSim 或者 Gazebo 进行 SITL 仿真时，我们可以以比实际时间流速更快或者更慢的时间流速运行仿真。

时间相对流速因子可通过环境变量 `PX4_SIM_SPEED_FACTOR` 进行设定。 例如，想以相对实际时间的 2 倍流速运行 jMAVSim 仿真：

    PX4_SIM_SPEED_FACTOR=2 make px4_sitl jmavsim
    

To run at half real-time:

    PX4_SIM_SPEED_FACTOR=0.5 make px4_sitl jmavsim
    

你也可以在当前会话（session）中使用 `EXPORT` 来将该因子应用于所有 SITL 仿真：

    export PX4_SIM_SPEED_FACTOR=2
    make px4_sitl jmavsim
    

> **Note** 在某些情况下当你的电脑的 IO 或者 CPU 性能不足以支撑仿真以设定的速度运行时，该速度会 “自动”降低。 性能强劲的台式机通常可以以 6-10 倍的流速运行仿真，而笔记本测通常可以实现 3-4 倍的流速。

### 启动脚本 {#scripts}

脚本被用于控制要使用的参数设置或要启动的模块。 它们位于 [ROMFS/px4fmu_common/init.d-posix](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d-posix) 目录中，`rcS` 文件是主要入口点。 有关详细信息，请参阅 [System startup](../concept/system_startup.md)。

## HITL 仿真环境

通过硬件在环（HITL）仿真使常规的 PX4 固件在真正的硬件上运行。 HITL 仿真环境记录于： [HITL 仿真](../simulation/hitl.md)。

## 操纵杆／手柄集成

*QGroundControl* 台式机版本可以连接到 USB Joystick/Gamepad，并通过 MAVLink 将其移动指令和按钮发送到 PX4。 这适用于 SITL 和 HITL 仿真，并允许直接控制仿真机。 如果你没有操纵杆，你也可以使用地面控制站的屏幕虚拟拇指杆来控制无人机。

有关设置信息，请参阅 *QGroundControl 用户指南 *：

* [操纵杆设置](https://docs.qgroundcontrol.com/en/SetupView/Joystick.html)
* [虚拟操纵杆](https://docs.qgroundcontrol.com/en/SettingsView/VirtualJoystick.html)

<!-- FYI Airsim info on this setting up remote controls: https://github.com/Microsoft/AirSim/blob/master/docs/remote_controls.md -->

## 相机模拟

PX4 支持在 [Gazebo](../simulation/gazebo.md) 模拟环境中捕获静止图像和视频。 这可以按照[ Gazebo> Video Streaming ](../simulation/gazebo.md#video-streaming)中的描述启用/设置。

这个模拟相机是一个实现 [MAVLink 相机协议的 gazebo 插件](https://mavlink.io/en/protocol/camera.html)。 PX4 与这个相机以 *exactly the same way* 连接／集成，与任何其他 MAVLink 相机一样：

1. [ TRIG_INTERFACE ](../advanced/parameter_reference.md#TRIG_INTERFACE)必须设置为` 3 `以配置相机触发驱动程序以与 MAVLink 相机一起使用 > **Tip**在此模式下，只要请求图像捕获，驱动程序就会发送[ CAMERA_TRIGGER ](https://mavlink.io/en/messages/common.html#CAMERA_TRIGGER)消息。 更多信息请参见：[Camera](https://docs.px4.io/en/peripherals/camera.html)。
2. PX4 必须在 GCS 和（模拟器）MAVLink Camera 之间转发所有摄像机命令。 您可以通过使用` -f `标志启动[ mavlink ](../middleware/modules_communication.md#mavlink)来执行此操作，如下所示，指定新连接的UDP端口。 ```mavlink start -u 14558 -o 14530 -r 4000 -f -m camera``` > **Note</ 0>不仅仅是摄像机将转发 MAVLink 消息，但摄像机将忽略它们认为不相关的消息。</li> </ol> 
    
    其他模拟器可以使用相同的方法来实现相机支持。
    
    ## 在远程服务器上运行仿真
    
    可以在一台计算机上运行模拟器，并从同一网络 (或具有适当路由的另一台网络) 上的另一台计算机访问模拟器。 例如，如果要测试在模拟车辆上运行的真实配套计算机硬件上运行的无人机应用程序，这可能很有用。
    
    这不是 "开箱即用" 的，因为 PX4 在默认情况下不会将数据包路由到外部接口 (以避免垃圾邮件和不同的模拟相互干扰)。 相反，它将数据包路由到 "本地主机"。
    
    有多种方法可以使 UDP 数据包在外部接口上可用，如下所述。
    
    ### 启用 MAV_BROADCAST
    
    启用 [MAV_BROADCAST](../advanced/parameter_reference.md#MAV_BROADCAST) 在本地网络上广播检测信号。
    
    然后，远程计算机可以通过侦听适当的端口 (即 *QGroundControl* 的14550端口) 连接到模拟器。
    
    ### 使用 MAVLink 路由器
    
    [mavlink-router](https://github.com/intel/mavlink-router) 可用于将数据包从本地主机路由到外部接口。
    
    ### 修改外部广播的配置
    
    默认情况下，[mavlink](../middleware/modules_communication.md#mavlink_usage) 模块路由到 *localhost*，但您可以指定要使用其 `-t` 选项广播的外部 IP 地址。
    
    这应该在各种配置文件中完成，其中调用了 `mavlink start`。 例如：[/ROMFS/px4fmu_common/init.d-posix/rcS](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d-posix/rcS)。
    
    ### SSH 通道
    
    ssh 是一个灵活的选项，因为模拟计算机和使用它的系统不需要在同一网络上。
    
    > **Note** 您也可以使用 vpn 向外部接口 (在同一网络或其他网络上) 提供隧道。
    
    创建隧道的一种方法是使用 ssh 隧道选项。 可以通过在 *localhost* 上运行以下命令来创建隧道，其中 `remote.local` 是远程计算机的名称：
    
        ssh -C -fR 14551:localhost:14551 remote.local
        
    
    UDP 数据包需要转换为 TCP 数据包，以便可以通过 ssh 对其进行路由。 [netcat](https://en.wikipedia.org/wiki/Netcat) 可以用于隧道的两边—首先转换 UDP 为 TCP 数据包，然后在另一端再转回 UDP 。
    
    > **Tip** 执行 *netcat* 之前， QGC 必须运行起来。
    
    在运行 QGC 的主机上，UDP 包的转换可以用以下命令实现：
    
        mkfifo /tmp/tcp2udp
        netcat -lvp 14551 < /tmp/tcp2udp | netcat -u localhost 14550 > /tmp/tcp2udp
        
    
    在 ssh 隧道的模拟器一端，命令是：
    
        mkfifo /tmp/udp2tcp
        netcat -lvup 14550 < /tmp/udp2tcp | netcat localhost 14551 > /tmp/udp2tcp
        
    
    端口号 `14550` 可以用于 QGroundControl 与其他的 GCS 连接，但应根据其他端点进行调整（比如开发者 API 等）。
    
    理论上，隧道可能无限期运行，但如果出现问题，可能需要重新启动 *netcat* 连接。
    
    [QGC_remote_connect.bash](https://raw.githubusercontent.com/ThunderFly-aerospace/sitl_gazebo/autogyro-sitl/scripts/QGC_remote_connect.bash) 脚本可以在 QGC 计算机上运行，以自动设置运行上述指令。 模拟必须已经在远程服务器上运行，并且您必须能够通过 ssh 登录到该服务器。