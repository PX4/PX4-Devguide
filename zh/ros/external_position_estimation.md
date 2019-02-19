# 利用视觉或运动捕捉系统进行位置估计

可视惯性测距（VIO）和运动捕捉（MOCAP）系统允许载具在全局位置源不可用或不可靠时（例如在室内，或在桥下飞行时）导航。 等等……

VIO 和 MOCAP 都从“视觉”信息中确定飞机的 *pose* （位置和姿态）。 它们之间的主要区别是框架透视图：

* VIO 使用 *板载传感器 * 从车辆的角度获取姿势数据（见 [egomotion](https://en.wikipedia.org/wiki/Visual_odometry#Egomotion)）。
* MoCap 使用 *离板摄像机* 系统在 3D 空间中获取飞机姿态数据（即它是一个外部系统，告诉飞机其姿态）。

任何类型系统的 Pose 数据都可用于更新基于 PX4 自动驾驶仪的局部位置估计（相对于本地源），也可以选择融合到飞机姿态估计中。

本主题介绍如何配置基于 px4 的系统，以便从 MoCap/VIO 系统（通过 ROS 或其他 MAVLink 系统）获取数据，更具体地说明如何设置 MoCap 系统，如 VICON 和 Optitrack，以及基于视觉的估计系统（如 [ROVIO](https://github.com/ethz-asl/rovio)、[SVO](https://github.com/uzh-rpg/rpg_svo) 和 [PTAM](https://github.com/ethz-asl/ethzasl_ptam)）。

> **Note** 说明因您使用的是 EKF2 还是 LPE 估计器而异。

## PX4 MAVLink 集成

PX4 使用以下 MAVLink 消息获取外部位置信息，并将其映射到 [uORB 主题](http://dev.px4.io/en/middleware/uorb.html)：

| MAVLink                                                                                                                                                                  | uORB                      |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------- |
| [VISION_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE)                                                                        | `vehicle_visual_odometry` |
| [ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY) (`frame_id =` [MAV_FRAME_VISION_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_VISION_NED)) | `vehicle_visual_odometry` |
| [ATT_POS_MOCAP](https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP)                                                                                              | `vehicle_mocap_odometry`  |
| [ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY) (`frame_id =` [MAV_FRAME_MOCAP_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_MOCAP_NED))   | `vehicle_mocap_odometry`  |

EKF2 只订阅 `vehicle_visual_odometry` 主题，因此只能处理前两个消息（MoCap 系统必须生成这些消息才能与 EKF2 配合使用）。 LPE 估计订阅所有主题，并且可以增强上面信息的所有进程。

> **Tip** PX4 默认使用 EKF2 估计。 相比 LPE 得到更好的测试和支持，更得到推荐。

消息应在 30Hz（如果包含协方差）和 50 Hz 之间进行流式传输。

以下 MAVLink 视觉消息暂不支持 PX4：[GLOBAL_VISION_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#GLOBAL_VISION_POSITION_ESTIMATE)，[VISION_SPEED_ESTIMATE](https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE)，[VICON_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#VICON_POSITION_ESTIMATE)

## 参考机架

在 MAVLink中，PX4 使用 FRD（X **F**orward, Y **R**ight and Z **D**own）机体坐标系，NED（X **N**orth, Y **E**ast, Z **D**own）世界坐标系；分别对应[MAV_FRAME_BODY_OFFSET_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_OFFSET_NED) 和 [MAV_FRAME_LOCAL_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_LOCAL_NED)。

根据您的源系统参考框架，您需要在发送 MAVLink Vision/MoCap 消息时应用自定义转换以获得适当的 NED 约定。

> **Tip** ROS 用户可以在下面的 [参考机架和 ROS](#ros_reference_frames) 中找到更详细的说明。

例如，如果使用 optitrack 框架，则本地框架在水平面上具有 $$x$$ 和 $$z$$（*x* 正面和 *z* 右），而 *y* 轴是垂直的，指向上方。 通过如下转换我们可以转换optrack坐标系到NED系中。

` x_{mav}`，` y_{mav}` 和 ` z_{mav}` 是我们将通过 MAVLink 发送的位置量，然后我们得到：

    x_{mav} = x_{mocap}
    y_{mav} = z_{mocap}
    z_{mav} = - y_{mocap}
    

在方向方面，保持标量部分 *w* 四元数，并以相同的方式交换矢量部分 *x*、*y* 和 *z*。 您可以将此技巧应用于每个系统-如果您需要获取 NED 帧，请相应地查看您的 MoCap 输出和交换轴。

## EKF2 调参/配置

必须将以下参数设置为将外部位置信息与 ekf2 一起使用（这些信息可以在 *QGroundControl* > **飞机设置参数 > ekf2** 中设置）。

| 参数                                                                                                                                                                                                            | 外部位置估计的设置                                                             |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------- |
| [EKF2_AID_MASK](../advanced/parameter_reference.md#EKF2_AID_MASK)                                                                                                                                           | 设置 *视觉位置合成* 和 *视觉偏航合成*                                                |
| [EKF2_HGT_MODE](../advanced/parameter_reference.md#EKF2_HGT_MODE)                                                                                                                                           | 设置为 *Vision* 使用视觉作为高度估计的主要来源。                                         |
| [EKF2_EV_DELAY](../advanced/parameter_reference.md#EKF2_EV_DELAY)                                                                                                                                           | 设置为测量的时间戳和 "实际" 捕获时间之间的差异。 有关详细信息，请参阅 [below](#tuning-EKF2_EV_DELAY)。 |
| [EKF2_EV_POS_X](../advanced/parameter_reference.md#EKF2_EV_POS_X), [EKF2_EV_POS_Y](../advanced/parameter_reference.md#EKF2_EV_POS_Y), [EKF2_EV_POS_Z](../advanced/parameter_reference.md#EKF2_EV_POS_Z) | 设置视觉传感器（或 MoCap 标记）相对于机器人的车身框架的位置。                                    |

> **Tip** 重新启动飞行控制器，以便参数更改生效。

#### 调参 EKF2_EV_DELAY {#tuning-EKF2_EV_DELAY}

[EKF2_EV_DELAY](../advanced/parameter_reference.md#EKF2_EV_DELAY) 是相对于 IMU 测量的 *Vision 位置估计延迟 *。

换句话说，它是视觉系统时间戳和 "实际" 捕获时间之间的差异，将记录的 IMU 时钟（"基本时钟" 为 ekf2）。

从技术上讲，如果 MoCap 和（例如）ROS 计算机之间有正确的时间戳（而不仅仅是到达时间）和时间同步（例如 NTP），则可以将其设置为0。 在现实中，这需要一些经验调整，因为整个 MoCap->PX4 链中的延迟是非常特定的。 系统设置完全同步链的情况很少见!

通过检查 IMU 速率和 EV 速率之间的偏移量，可以从日志中获得延迟的粗略估计：

![ekf2_ev_delay 日志](../../assets/ekf2/ekf2_ev_delay_tuning.png)

> **Note** 外部数据图表 与 可使用 [FlightPlot](https://docs.px4.io/en/log/flight_log_analysis.html#flightplot-desktop) 或类似的飞行分析工具生成机载估计（如上）。

该值可以通过不同的参数一起调整，在动态变化中来保证最低 EKF 。

## LPE 调参/配置

首先需要通过设置 [SYS_MC_EST_GROUP](../advanced/parameter_reference.md#SYS_MC_EST_GROUP) 参数进行 [switch lpe 估计值 ](../advanced/switching_state_estimators.md)。

> **Note** 如果定位 `px4_fmu-v2` 硬件，则还需要使用包含 LPE 模块的固件版本（其他 FMU 系列硬件的固件包括 LPE 和 EKF）。 LPE 版本可以在每个 PX4 版本的 zip 文件中找到，也可以使用生成命令 `make px4_fmu-v2_lpe` 从源生成。 有关详细信息, 请参阅 [ Building the code ](../setup/building_px4.md)。

### 启用外部位置输入

必须将以下参数设置为将外部位置信息与 LPE 一起使用（这些信息可以在 *QGroundControl* > > **Vehicle 设置 > 参数 > 本地位置估计 </1 > 中设置）。</p> 

| 参数                                                                  | 外部位置估计的设置                                                |
| ------------------------------------------------------------------- | -------------------------------------------------------- |
| [LPE_FUSION](../advanced/parameter_reference.md#LPE_FUSION)         | 如果选中了 *fuse 视觉位置 *（默认情况下启用），则启用视觉集成。                     |
| [ATT_EXT_HDG_M](../advanced/parameter_reference.md#ATT_EXT_HDG_M) | 设置为1或 2，以启用外部标题集成。 将其设置为1将启用视觉，而2则启用了 MoCap heading 的使用。 |

### 禁用气压计融合

如果从 VIO 或 MoCap 信息中已经提供了高度精确的高度，则禁用 LPE 中的巴洛校正以减少 z 轴上的漂移可能会很有用。

这可以通过 *QGroundControl* 中通过取消选中 [LPE_FUSION](../advanced/parameter_reference.md#LPE_FUSION) 参数中的 *fuse baro</0 > 选项来实现。</p> 

### 滤波噪声参数调参

如果您的视觉或 MoCap 数据非常准确，并且您只希望估计器对其进行严格跟踪, 则应减少标准偏差参数、[ LPE_VIS_XY ](../advanced/parameter_reference.md#LPE_VIS_XY) 和 [ LPE_VIS_Z ](../advanced/parameter_reference.md#LPE_VIS_Z) (用于视觉) 或 [ LPE_VIC_P ](../advanced/parameter_reference.md#LPE_VIC_P)（用于 MoCap）。 减小它们会使估计器更加信任外部传入的位姿信息。 您可能需要将它们设置为允许的最小值。

> **Tip** 如果性能仍然较差，请尝试增大 [ LPE_PN_V](../advanced/parameter_reference.md#LPE_PN_V) 参数。 这将使估计器在估计速度时更信任测量值。

## 使用 ROS

ROS 不是提供外部姿态信息的 *required*，但强烈建议使用它，因为它已经与 VIO 和 MoCap 系统进行了良好的集成。 PX4 必须已设置如上所示。

### 将数据输入 ROS

VIO 和 MoCap 系统具有不同的获取姿势数据的方式，并且有自己的设置和主题。

[below](#setup_specific_systems) 涵盖了特定系统的设置。 对于其他系统，请参阅供应商设置文档。

### 将数据回传给 PX4 {#relaying_pose_data_to_px4}

MAVROS 具有插件，可使用以下管道从 VIO 或 MOCAP 系统中继可视化估计：

| ROS                      | MAVLink                                                                                                                                                                  | uORB                      |
| ------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------- |
| /mavros/vision_pose/pose | [VISION_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE)                                                                        | `vehicle_visual_odometry` |
| /mavros/odometry/odom    | [ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY) (`frame_id =` [MAV_FRAME_VISION_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_VISION_NED)) | `vehicle_visual_odometry` |
| /mavros/mocap/pose       | [ATT_POS_MOCAP](https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP)                                                                                              | `vehicle_mocap_odometry`  |
| /mavros/odometry/odom    | [ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY) (`frame_id =` [MAV_FRAME_MOCAP_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_MOCAP_NED))   | `vehicle_mocap_odometry`  |

您可以将上述任何管道与 LPE 一起使用。

如果您使用的是 EKF2，则仅支持 "视觉" 管道。 要将 MoCap 数据与 EKF2 一起使用，您必须 [remap](http://wiki.ros.org/roslaunch/XML/remap) 从 mocap 获得的位置信息主题：

* 必须重新映射 `geometry_msgs/PoseStamped` 或 `geometry_msgs/PoseWithCovarianceStamped` 类型的 MOCAP ROS 主题，以 `/mavros/vision_pose/pose`。 `geometry_msgs/PoseStamped` 主题是最常见的，因为 mocap 通常没有与数据相关的协方差。
* 如果您通过 `nav_msgs/Odometry` ROS 消息获取数据，则需要重新映射数据以 `/mavros/odometry/odom`。

### 参考框架和 ROS {#ros_reference_frames}

ROS 和 PX4 使用的本地/世界坐标系和全局框架是不同的。

| 框架    | ROS                                                                                  | PX4                                           |
| ----- | ------------------------------------------------------------------------------------ | --------------------------------------------- |
| 机体    | FLU (x < 0>F</strong>orward、y < 0>L</strong>eft、z < 0>U</strong>p), 通常命名 `base_link` | FRD (X **F**orward, Y **R**ight 和 Z **D**own) |
| 世界坐标系 | Enu (x < 0>E</strong>ast、y < 0>N</strong>orth 和 z up), 命名 `odom` 或 `map`             | NED (X **N**orth, Y **E**ast, Z **D**own)     |

> **Tip** 有关 ROS 框架的详细信息，请参阅 [REP105: Coordinate Frames for Mobile Platforms](http://www.ros.org/reps/rep-0105.html)。

两个帧都显示在下图中（NED 在左/ENU 在右）。

![参考机架](../../assets/lpe/ref_frames.png)

在使用外部航向估计时，磁北将被忽略，并使用与世界坐标系 *x* 轴相对应的矢量进行伪造 (可在 vision/mocap 校准期间自由放置)。 因此，本地 *x* 给出了 yaw 角。

> **Note**在 mocap 软件中创建刚体时，请记住首先将机器人的本地坐标系 *x* 轴与世界坐标系 *x* 轴对齐，否则偏航估计将有初始偏移量。

利用MAVROS功能包，以上操作会十分简单。 ROS 默认使用 ENU 系, 因此你在MAVROS中所有代码必须遵循ENU系。 如果您有一个 Optitrack 系统, 则可以使用 [ mocap_optitrack ](https://github.com/ros-drivers/mocap_optitrack) 节点, 其已经发布了一个关于刚体位姿的一个ROS话题。 通过重新映射，您可以直接将其发布在 `mocap_pose_estimate` 因为它没有任何转换，mavros 将负责 NED 转换。

## 特定的系统设置 {#setup_specific_systems}

### 光学跟踪 MoCap

以下步骤说明如何将位置估计从 [OptiTrack](http://optitrack.com/systems/#robotics) 系统输入到 px4。 假定 mocap 系统已校准。 有关校准过程的教程，请参阅 [this video ](https://www.youtube.com/watch?v=cNZaFEghTBU)。

#### 设置 *Motive* mocap 软件

* 将无人机的前进方向与 [system + x-axiss](https://v20.wiki.optitrack.com/index.php?title=Template:Coordinate_System) 对齐
* [Define a rigid body in the Motive software](https://www.youtube.com/watch?v=1e6Qqxqe-k0)。 为机器人指定一个不包含空格的名称，例如 `robot1` 而不是 `Rigidbody 1 `
* [启用帧广播和 VRPN 流](https://www.youtube.com/watch?v=yYRNG58zPFo)
* 将 "向上" 轴设置为 z 轴（默认值为 y）

#### 将数据输入 ROS

* 安装 `vrpn_client_ros` 包
* 你可以通过运行在一个单独的话题上得到每一个机体的姿势 
        bash
        roslaunch vrpn_client_ros sample.launch server:=<mocap machine ip>

如果你把机体命名为 `robot1`，你会得到一个主题，比如 `/vrpn_client_node/robot1/pose`

#### 重新映射/重新映射位置数据

MAVROS 提供了一个插件来中继在 `/mavros/vision_pose/pose` 上发布的姿势数据到 px4。 假设 mavros 正在运行，您只需 **remap** 从 mcap 获得的位置主题 `/vrpn_client_node/&lt;rigid_body_name&gt;/pose` 直接到 `/mavros/vision_pose/pose`。 请注意，mavros 还提供了一个 `mocap` 主题，用于将 `ATT_POS_MOCAP` 提供给 px4，但它不适用于 ekf2。 但是，它适用于 lpe。

> **Note** 上文介绍了 [Relaying 将数据设置为 px4](#relaying_pose_data_to_px4) （`/vrpn_client_node/&lt;rigid_body_name&gt;/pose` 是 `geometry_msgs/PoseStamped`的类型）。

假设您已按上述方式配置了 EKF2 参数，那么现在就设置并融合了 MoCap 数据。

您现在已准备好继续进行第一次飞行。

## 第一次飞行

在设置了上述（特定）系统之一之后，您现在应该可以进行测试了。 下面的说明显示了如何对 MoCap 和 VIO 系统执行此操作

### MoCap 首飞

请检查

* 创建机体之 **前**，将飞机与世界坐标系 x 轴对齐。
* 通过 mavlink 并检查带有 *QGroundControl* 的 mavlink 检查器，本地姿势主题在 NED 中。
* 用手移动机器人，看看估计的本地坐标位置是否一致（总是在 NED 中）。
* 在垂直轴上旋转机器人，并使用 mavlink 检查器检查出 yaw。

如果以上步骤没问题，你可以开始你的第一次飞行。

将无人机放在地面上，开启流媒体 MoCap 反馈。 油门杆推到最低并解锁。

此时，设置为位置控制模式。 如果切换成功，飞控会闪绿灯。 绿灯代表：你的外部位置信息已经注入到飞控中，并且位置控制模式已经切换成功。

油门杆居中，这是油门控制死区。 如果在死区中，则无人机会保持其当前高度。往上推杆，则会上升，往下推杆，则会下降。 同理对于另一个杆。

推油门杆，则无人机会起飞，起飞后，立即将其拉回中位。 检查此时无人机能否悬停。

如果这一切都没问题，那么你可以开始进行offboard模式下的试验了（发布自行设定的位置期望值给飞控）。

### VIO 首飞

待定。