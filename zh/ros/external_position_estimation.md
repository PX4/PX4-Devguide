# Using Vision or Motion Capture Systems

> 在开始下面这段教程之前，请确保你的飞控是一个使能了LPE模块的固件版本。 The LPE version of the PX4 firmware can be found inside the zip file of the latest PX4 release or it can be built from source using a build command such as `make px4_fmu-v2_lpe`. 有关详细信息, 请参阅 [ Building the code ](../setup/building_px4.md)。

本页的目的是为了让PX4固件获得除 GPS 以外的位置数据 (比如像VICON和 Optitrack 等动作捕捉系统和基于视觉的位置估计系统 (如 [ ROVIO ](https://github.com/ethz-asl/rovio)、[ SVO ](https://github.com/uzh-rpg/rpg_svo) 或 [ PTAM ](https://github.com/ethz-asl/ethzasl_ptam))）

位置信息可以来自一个机载电脑或者板外设备（比如VICON） 这些数据用来更新飞控相对于原点的本地位置信息。 来自视觉估计或运动捕捉系统的偏航角也可以用于姿态估计器中。

这一套系统可用于室内悬停或基于视觉的航点导航等应用。

对于视觉, 用于发送姿势数据的 MAVLink 消息是 [ VISION_POSITION_ESTIMATE ](https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE), 而对于运动捕获系统的发送的的 MAVLink 消息是 [ ATT_POS_MOCAP ](https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP) 。

mavros功能包中已经包含了这些mavlink消息的发送和接收接口。 这些消息也可以直接使用MAVLINK库并编写C/C++代码来发送和接收。 对应的ROS 主题是: 用于 mocap 系统的 ` mocap_pose_estimate ` 和用于视觉的 ` vision_pose_estimate `。 有关详细信息, 请检查 [ mavros_extras ](http://wiki.ros.org/mavros_extras)。

**此功能只能与 LPE 估计器一起使用。**

## 针对Vision和Mocap的LPE模块调参

### 使能外部位置和姿态输入

你需要去设置一些参数（利用QGC或者NSH）去启用或关闭是否使用Vision/mocap提供的信息的功能。

参看系统参数`ATT_EXT_HDG_M`设置为1或2来使能融合外部航向角。 设置为1则会使能融合视觉信息，设置为2则会使能mocap信息。

LPE中会默认融合视觉信息。 你利用QGC可以设置参数`LPE_FUSION` 。 并且请确认检查过了fuse vision position。

#### 关闭气压计数据融合

如果从视觉或 mocap 信息中可以获得准确的高度信息, 则在 LPE 中禁用融合气压计数据以减少 Z 轴上的漂移。

同样，通过QGC来设置`LPE_FUSION`中的一个位来实现。 Just uncheck "fuse baro".

#### 滤波噪声参数调参

如果您的视觉或 mocap 数据非常准确, 并且您只希望估计器对其进行严格跟踪, 则应减少标准偏差参数、` LPE_VIS_XY ` 和 ` LPE_VIS_Z ` (用于视觉) 或 ` LPE_VIC_P ` (用于运动捕获)。 减小它们会使估计器更加信任外部传入的位姿信息。 您可能需要将它们设置为允许的最小值。

> ** 提示 **如果性能仍然较差, 请尝试增大` LPE_PN_V ` 参数。 这将使估计器在估计速度时更信任测量值。

## 关于坐标系

本节演示如何使用适当的参考坐标系。 关于坐标系有各种各样的表示, 但我们将使用其中两个: ENU 和 NED。

* ENU系：X轴指向东，Y指向北，Z指向天。 相应的机体系：X指向前，Z指向上，Y则顺应右手法则。

* NED系：X轴指向北，Y轴指向东，Z指向地。 相应的机体系：X指向前，Z指向下，Y则顺应右手法则。

各坐标系如下图所示，NED在左边，ENU在右边。

![参考机架](../../assets/lpe/ref_frames.png)

当使用外部信息作为航向标准时，地磁北将会被忽略。注意：你可以轻易设置外部的偏航角0度值，这时X轴会指向你自定义的X轴。

> ** 信息 **在 mocap系统的地面站 软件中创建刚体时, 请记住首先将机器人的本地 * x * 轴与世界 * x * 轴对齐, 否则偏航估计将具有初始偏移量。

### 利用MAVROS来实现这些

利用MAVROS功能包，以上操作会十分简单。 ROS 默认使用 ENU 系, 因此你在MAVROS中所有代码必须遵循ENU系。 如果您有一个 Optitrack 系统, 则可以使用 [ mocap_optitrack ](https://github.com/ros-drivers/mocap_optitrack) 节点, 其已经发布了一个关于刚体位姿的一个ROS话题。 通过重新映射（方向转换）, 您可以直接利用` mocap_pose_estimate ` 插件来发布它, 请注意你需要遵循ENU系。

### 不使用MAVROS的方法

如果你不想使用MAVROS或者ROS，你需要发送`ATT_POS_MOCAP` 这条MAVLINK消息给飞控。 这种情况下，请注意方向问题，请遵循飞控内部的NED系。

让我们以 Optitrack 为例;如果我们设置optitrack的坐标系设置为x指向前方，z指向右，y竖直向上 通过如下转换我们可以转换optrack坐标系到NED系中。

* x_{mav}*, * y_{mav}* 和 * z_{mav}* 是我们将通过 MAVLink 发送的位置量, 然后我们得到:

*x_{mav}* = *x_{mocap}* *y_{mav}* = *z_{mocap}* *z_{mav}* = - *y_{mocap}*

Regarding the orientation, keep the scalar part *w* of the quaternion the same and swap the vector part *x*, *y* and *z* in the same way. 上述的转换技巧你可以应用于任何系统：对比一下NED系和你的外部消息的坐标系你就会知道该如何转换。

## 第一次飞行

首先你需要检查你的各项设置。

请检查

* **Before** creating the rigid body, align the robot with world x axis
* 通过MAVLINK发送相应的消息，并在QGC监控该消息。
* 用手简单移动无人机来检查估计器发布的位置方向是否正确。
* 旋转无人机来检查偏航角方向是否正确。

如果以上步骤没问题，你可以开始你的第一次飞行。

将无人机摆放在地面，并启用mocap系统。 油门杆推到最低并解锁。

此时，设置为位置控制模式。 如果切换成功，飞控会闪绿灯。 绿灯代表：你的外部位置信息已经注入到飞控中，并且位置控制模式已经切换成功。

油门杆居中，这是油门控制死区。 如果在死区中，则无人机会保持其当前高度。往上推杆，则会上升，往下推杆，则会下降。 同理对于另一个杆。

推油门杆，则无人机会起飞，起飞后，立即将其拉回中位。 检查此时无人机能否悬停。

如果这一切都没问题，那么你可以开始进行offboard模式下的试验了（发布自行设定的位置期望值给飞控）。