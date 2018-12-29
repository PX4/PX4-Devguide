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

The messages should be streamed at between 30Hz (if containing covariances) and 50 Hz.

The following MAVLink "vision" messages are not currently supported by PX4: [GLOBAL_VISION_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#GLOBAL_VISION_POSITION_ESTIMATE), [VISION_SPEED_ESTIMATE](https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE), [VICON_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#VICON_POSITION_ESTIMATE)

## Reference Frames

PX4 uses FRD (X **F**orward, Y **R**ight and Z **D**own) for the local body frame, and NED (X **N**orth, Y **E**ast, Z **D**own) for the local world frame - set in MAVLink using [MAV_FRAME_BODY_OFFSET_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_OFFSET_NED) and [MAV_FRAME_LOCAL_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_LOCAL_NED), respectively.

Depending on your source your source system reference frame you will need to apply a custom transformation to obtain the appropriate NED convention when sending the MAVLink Vision/MoCap messages.

> **Tip** ROS users can find more detailed instructions below in [Reference Frames and ROS](#ros_reference_frames).

For example, if using the Optitrack framework the local frame has $$x$$ and $$z$$ on the horizontal plane (*x* front and *z* right) while *y* axis is vertical and pointing up. 通过如下转换我们可以转换optrack坐标系到NED系中。

If `x_{mav}`, `y_{mav}` and `z_{mav}` are the coordinates that are sent through MAVLink as position feedback, then we obtain:

    x_{mav} = x_{mocap}
    y_{mav} = z_{mocap}
    z_{mav} = - y_{mocap}
    

Regarding the orientation, keep the scalar part *w* of the quaternion the same and swap the vector part *x*, *y* and *z* in the same way. You can apply this trick with every system - if you need to obtain a NED frame, look at your MoCap output and swap axis accordingly.

## EKF2 Tuning/Configuration

The following parameters must be set to use external position information with EKF2 (these can be set in *QGroundControl* > **Vehicle Setup > Parameters > EKF2**).

| 参数                                                                                                                                                                                                            | Setting for External Position Estimation                                                                                                               |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------ |
| [EKF2_AID_MASK](../advanced/parameter_reference.md#EKF2_AID_MASK)                                                                                                                                           | Set *vision position fusion* and *vision yaw fusion*                                                                                                   |
| [EKF2_HGT_MODE](../advanced/parameter_reference.md#EKF2_HGT_MODE)                                                                                                                                           | Set to *Vision* to use the vision a primary source for altitude estimation.                                                                            |
| [EKF2_EV_DELAY](../advanced/parameter_reference.md#EKF2_EV_DELAY)                                                                                                                                           | Set to the difference between the timestamp of the measurement and the "actual" capture time. For more information see [below](#tuning-EKF2_EV_DELAY). |
| [EKF2_EV_POS_X](../advanced/parameter_reference.md#EKF2_EV_POS_X), [EKF2_EV_POS_Y](../advanced/parameter_reference.md#EKF2_EV_POS_Y), [EKF2_EV_POS_Z](../advanced/parameter_reference.md#EKF2_EV_POS_Z) | Set the position of the vision sensor (or MoCap markers) with respect to the robot's body frame.                                                       |

> **Tip** Reboot the flight controller in order for parameter changes to take effect.

#### Tuning EKF2_EV_DELAY {#tuning-EKF2_EV_DELAY}

[EKF2_EV_DELAY](../advanced/parameter_reference.md#EKF2_EV_DELAY) is the *Vision Position Estimator delay relative to IMU measurements*.

Or in other words, it is the difference between the vision system timestamp and the "actual" capture time that would have been recorded by the IMU clock (the "base clock" for EKF2).

Technically this can be set to 0 if there is correct timestamping (not just arrival time) and timesync (e.g NTP) between MoCap and (for example) ROS computers. In reality, this needs some empirical tuning since delays in the entire MoCap->PX4 chain are very setup-specific. It is rare that a system is setup with an entirely synchronised chain!

A rough estimate of the delay can be obtained from logs by checking the offset between IMU rates and the EV rates:

![ekf2_ev_delay log](../../assets/ekf2/ekf2_ev_delay_tuning.png)

> **Note** A plot of external data vs. onboard estimate (as above) can be generated using [FlightPlot](https://docs.px4.io/en/log/flight_log_analysis.html#flightplot-desktop) or similar flight analysis tools.

The value can further be tuned by varying the parameter to find the value that yields the lowest EKF innovations during dynamic maneuvers.

## LPE Tuning/Configuration

You will first need to [switch to the LPE estimator](../advanced/switching_state_estimators.md) by setting the [SYS_MC_EST_GROUP](../advanced/parameter_reference.md#SYS_MC_EST_GROUP) parameter.

> **Note** If targeting `px4_fmu-v2` hardware you will also need to use a firmware version that includes the LPE module (firmware for other FMU-series hardware includes both LPE and and EKF). The LPE version can be found in the zip file for each PX4 release or it can be built from source using the build command `make px4_fmu-v2_lpe`. 有关详细信息, 请参阅 [ Building the code ](../setup/building_px4.md)。

### Enabling External Pose Input

The following parameters must be set to use external position information with LPE (these can be set in *QGroundControl* > **Vehicle Setup > Parameters > Local Position Estimator**).

| 参数                                                                  | Setting for External Position Estimation                                                                                               |
| ------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------- |
| [LPE_FUSION](../advanced/parameter_reference.md#LPE_FUSION)         | Vision integration is enabled if *fuse vision position* is checked (it is enabled by default).                                         |
| [ATT_EXT_HDG_M](../advanced/parameter_reference.md#ATT_EXT_HDG_M) | Set to 1 or 2 to enable external heading integration. Setting it to 1 will cause vision to be used, while 2 enables MoCap heading use. |

### Disabling Barometer Fusion

If a highly accurate altitude is already available from VIO or MoCap information, it may be useful to disable the baro correction in LPE to reduce drift on the Z axis.

This can be done by in *QGroundControl* by unchecking the *fuse baro* option in the [LPE_FUSION](../advanced/parameter_reference.md#LPE_FUSION) parameter.

### 滤波噪声参数调参

If your vision or MoCap data is highly accurate, and you just want the estimator to track it tightly, you should reduce the standard deviation parameters: [LPE_VIS_XY](../advanced/parameter_reference.md#LPE_VIS_XY) and [LPE_VIS_Z](../advanced/parameter_reference.md#LPE_VIS_Z) (for VIO) or [LPE_VIC_P](../advanced/parameter_reference.md#LPE_VIC_P) (for MoCap). 减小它们会使估计器更加信任外部传入的位姿信息。 您可能需要将它们设置为允许的最小值。

> **Tip** If performance is still poor, try increasing the [LPE_PN_V](../advanced/parameter_reference.md#LPE_PN_V) parameter. This will cause the estimator to trust measurements more during velocity estimation.

## Working with ROS

ROS is not *required* for supplying external pose information, but is highly recommended as it already comes with good integrations with VIO and MoCap systems. PX4 must already have been set up as above.

### Getting Pose Data Into ROS

VIO and MoCap systems have different ways of obtaining pose data, and have their own setup and topics.

The setup for specific systems is covered [below](#setup_specific_systems). For other systems consult the vendor setup documentation.

### Relaying Pose Data to PX4 {#relaying_pose_data_to_px4}

MAVROS has plugins to relay a visual estimation from a VIO or MoCap system using the following pipelines:

| ROS                      | MAVLink                                                                                                                                                                  | uORB                      |
| ------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------- |
| /mavros/vision_pose/pose | [VISION_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE)                                                                        | `vehicle_visual_odometry` |
| /mavros/odometry/odom    | [ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY) (`frame_id =` [MAV_FRAME_VISION_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_VISION_NED)) | `vehicle_visual_odometry` |
| /mavros/mocap/pose       | [ATT_POS_MOCAP](https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP)                                                                                              | `vehicle_mocap_odometry`  |
| /mavros/odometry/odom    | [ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY) (`frame_id =` [MAV_FRAME_MOCAP_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_MOCAP_NED))   | `vehicle_mocap_odometry`  |

You can use any of the above pipelines with LPE.

If you're working with EKF2, only the "vision" pipelines are supported. To use MoCap data with EKF2 you will have to [remap](http://wiki.ros.org/roslaunch/XML/remap) the pose topic that you get from MoCap:

* MoCap ROS topics of type `geometry_msgs/PoseStamped` or `geometry_msgs/PoseWithCovarianceStamped` must be remapped to `/mavros/vision_pose/pose`. The `geometry_msgs/PoseStamped` topic is most common as MoCap doesn't usually have associated covariances to the data.
* If you get data through a `nav_msgs/Odometry` ROS message then you will need to remap it to `/mavros/odometry/odom`.

### Reference Frames and ROS {#ros_reference_frames}

The local/world and world frames used by ROS and PX4 are different.

| Frame | ROS                                                                           | PX4                                             |
| ----- | ----------------------------------------------------------------------------- | ----------------------------------------------- |
| Body  | FLU (X **F**orward, Y **L**eft, Z **U**p), usually named `base_link`          | FRD (X **F**orward, Y **R**ight and Z **D**own) |
| World | ENU (X **E**ast, Y **N**orth and Z Up), with the naming being `odom` or `map` | NED (X **N**orth, Y **E**ast, Z **D**own)       |

> **Tip** See [REP105: Coordinate Frames for Mobile Platforms](http://www.ros.org/reps/rep-0105.html) for more information about ROS frames.

Both frames are shown in the image below (NED on left/ENU on right).

![参考机架](../../assets/lpe/ref_frames.png)

When using external heading estimation, magnetic North is ignored and faked with a vector corresponding to world *x* axis (which can be placed freely during Vision/MoCap calibration). Yaw angle is therefore given with respect to local *x*.

> **Note** When creating the rigid body in the MoCap software, remember to first align the robot's local *x* axis with the world *x* axis otherwise yaw estimation will have an initial offset.

Using MAVROS, this operation is straightforward. ROS 默认使用 ENU 系, 因此你在MAVROS中所有代码必须遵循ENU系。 如果您有一个 Optitrack 系统, 则可以使用 [ mocap_optitrack ](https://github.com/ros-drivers/mocap_optitrack) 节点, 其已经发布了一个关于刚体位姿的一个ROS话题。 With a remapping you can directly publish it on `mocap_pose_estimate` as it is without any transformation and MAVROS will take care of NED conversions.

## Specific System Setups {#setup_specific_systems}

### OptiTrack MoCap

The following steps explain how to feed position estimates from an [OptiTrack](http://optitrack.com/systems/#robotics) system to PX4. It is assumed that the MoCap system is calibrated. See [this video](https://www.youtube.com/watch?v=cNZaFEghTBU) for a tutorial on the calibration process.

#### Steps on the *Motive* MoCap software

* Align your robot's forward direction with the the [system +x-axis](https://v20.wiki.optitrack.com/index.php?title=Template:Coordinate_System)
* [Define a rigid body in the Motive software](https://www.youtube.com/watch?v=1e6Qqxqe-k0). Give the robot a name that does not contain spaces, e.g. `robot1` instead of `Rigidbody 1`
* [Enable Frame Broadacst and VRPN streaming](https://www.youtube.com/watch?v=yYRNG58zPFo)
* Set the Up axis to be the Z axis (the default is Y)

#### Getting pose data into ROS

* Install the `vrpn_client_ros` package
* You can get each rigid body pose on an individual topic by running 
        bash
        roslaunch vrpn_client_ros sample.launch server:=<mocap machine ip>

If you named the rigidbody as `robot1`, you will get a topic like `/vrpn_client_node/robot1/pose`

#### Relaying/remapping Pose Data

MAVROS provides a plugin to relay pose data published on `/mavros/vision_pose/pose` to PX4. Assuming that MAVROS is running, you just need to **remap** the pose topic that you get from MoCap `/vrpn_client_node/<rigid_body_name>/pose` directly to `/mavros/vision_pose/pose`. Note that there is also a `mocap` topic that MAVROS provides to feed `ATT_POS_MOCAP` to PX4, but it is not applicable for EKF2. However, it is applicable with LPE.

> **Note** Remapping pose topics is covered above [Relaying pose data to PX4](#relaying_pose_data_to_px4) (`/vrpn_client_node/<rigid_body_name>/pose` is of type `geometry_msgs/PoseStamped`).

Assuming that you have configured EKF2 parameters as described above, PX4 now is set and fusing MoCap data.

You are now set to proceed to the first flight.

## 第一次飞行

After setting up one of the (specific) systems described above you should now be ready to test. The instructions below show how to do so for MoCap and VIO systems

### MoCap First Flight

请检查

* **Before** creating the rigid body, align the robot with world x axis.
* Stream over MAVLink and check the MAVLink inspector with *QGroundControl*, the local pose topic should be in NED.
* Move the robot around by hand and see if the estimated local position is consistent (always in NED).
* Rotate the robot on the vertical axis and check the yaw with the MAVLink inspector.

如果以上步骤没问题，你可以开始你的第一次飞行。

Put the robot on the ground and start streaming MoCap feedback. 油门杆推到最低并解锁。

此时，设置为位置控制模式。 如果切换成功，飞控会闪绿灯。 绿灯代表：你的外部位置信息已经注入到飞控中，并且位置控制模式已经切换成功。

油门杆居中，这是油门控制死区。 如果在死区中，则无人机会保持其当前高度。往上推杆，则会上升，往下推杆，则会下降。 同理对于另一个杆。

推油门杆，则无人机会起飞，起飞后，立即将其拉回中位。 检查此时无人机能否悬停。

如果这一切都没问题，那么你可以开始进行offboard模式下的试验了（发布自行设定的位置期望值给飞控）。

### VIO First Flight

TBD.