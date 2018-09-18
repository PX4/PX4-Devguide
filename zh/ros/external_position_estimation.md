# Using Vision or Motion Capture Systems

> 在开始下面这段教程之前，请确保你的飞控是一个使能了LPE模块的固件版本。 PX4 固件的 LPE 版本可以在最新的 PX4 发行版的 zip 文件中找到, 也可以使用生成命令 (如 ` build px4fmu-v2_lpe `) 从源生成。 有关详细信息, 请参阅 [ Building the code ](../setup/building_px4.md)。

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

* ENU系：X轴指向东，Y指向北，Z指向天。 Robot frame is *x* towards the front, *z* up and *y* accordingly.

* NED has *x* towards North, *y* East and *z* down. Robot frame is *x* towards the front, *z* down and *y* accordingly.

Frames are shown in the image below: NED on the left while ENU on the right.

![Reference frames](../../assets/lpe/ref_frames.png)

With the external heading estimation, however, magnetic North is ignored and faked with a vector corresponding to world *x* axis (which can be placed freely at mocap calibration); yaw angle will be given with respect to local *x*.

> **Info** When creating the rigid body in the mocap software, remember to first align the robot's local *x* axis with the world *x* axis otherwise yaw estimation will have an initial offset.

### Using Mavros

With MAVROS this operation is straightforward. ROS uses ENU frames as convention, therefore position feedback must be provided in ENU. If you have an Optitrack system you can use [mocap_optitrack](https://github.com/ros-drivers/mocap_optitrack) node which streams the object pose on a ROS topic already in ENU. With a remapping you can directly publish it on `mocap_pose_estimate` as it is without any transformation and mavros will take care of NED conversions.

### Without Mavros

If you do not use MAVROS or ROS in general, you need to stream data over MAVLink with `ATT_POS_MOCAP` message. In this case you will need to apply a custom transformation depending on the system in order to obtain NED convention.

Let us take as an example the Optitrack framework; in this case the local frame has $$x$$ and $$z$$ on the horizontal plane (*x* front and *z* right) while *y* axis is vertical and pointing up. A simple trick is swapping axis in order to obtained NED convention.

We call *x_{mav}*, *y_{mav}* and *z_{mav}* the coordinates that are sent through MAVLink as position feedback, then we obtain:

*x_{mav}* = *x_{mocap}* *y_{mav}* = *z_{mocap}* *z_{mav}* = - *y_{mocap}*

Regarding the orientation, keep the scalar part *w* of the quaternion the same and swap the vector part *x*, *y* and *z* in the same way. You can apply this trick with every system; you need to obtain a NED frame, look at your mocap output and swap axis accordingly.

## First Flight

At this point, if you followed those steps, you are ready to test your setup.

Be sure to perform the following checks:

* **Before** creating the rigid body, align the robot with world x axis
* Stream over MAVLink and check the MAVLink inspector with QGroundControl, the local pose topic should be in NED
* Move the robot around by hand and see if the estimated local position is consistent (always in NED)
* Rotate the robot on the vertical axis and check the yaw with the MAVLink inspector

If those steps are consistent, you can try your first flight.

Put the robot on the ground and start streaming mocap feedback. Lower your left (throttle) stick and arm the motors.

At this point, with the left stick at the lowest position, switch to position control. You should have a green light. The green light tells you that position feedback is available and position control is now activated.

Put your left stick at the middle, this is the dead zone. With this stick value, the robot maintains its altitude; raising the stick will increase the reference altitude while lowering the value will decrease it. Same for right stick on x and y.

Increase the value of the left stick and the robot will take off, put it back to the middle right after. Check if it is able to keep its position.

If it works, you may want to set up an [offboard](offboard_control.md) experiment by sending position-setpoint from a remote ground station.