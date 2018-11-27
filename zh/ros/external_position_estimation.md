---
translated_page: https://github.com/PX4/Devguide/blob/master/en/ros/external_position_estimation.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# Using Vision or Motion Capture systems

> **知悉：** Before following the instructions below, ensure that your autopilot has a firmware version with the LPE modules enabled. The LPE version of the PX4 firmware can be found inside the zip file of the latest PX4 release or it can be built from source using a build command such as `make px4_fmu-v2_lpe`. See [Building the Code](../setup/building_px4.md) for more details.

本文旨在使用除GPS之外的位置数据源构建一个基于PX4的系统，例如像VICON、Optitrack之类的运动捕捉系统和像[ROVIO](https://github.com/ethz-asl/rovio)、[SVO](https://github.com/uzh-rpg/rpg_svo)或者[PTAM](https://github.com/ethz-asl/ethzasl_ptam)之类的基于视觉的估计系统。

位置估计既可以来源于板载计算机，也可以来源于外部系统（例如：VICON）。这些数据用于更新机体相对于本地坐标系的位置估计。来自于视觉或者运动捕捉系统的朝向信息也可以被适当整合进姿态估计器中。

现在，这个系统被用来进行室内位置控制或者基于视觉的路径点导航。

对于视觉，用来发送位姿数据的MAVLink消息是[VISION_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE)。对于运动捕捉系统，相应的则为[ATT_POS_MOCAP](https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP)。

默认发送这些消息的应用是ROS-Mavlink接口MAVROS，当然，也可以直接使用纯C/C++代码或者MAVLink()库来发送它们。

## LPE Tuning for Vision or Mocap

### 使能外部位姿输入

需要设置几个参数（从QGroundControl或者NSH shell）来使能或者禁用视觉/运动捕捉。


> 设置系统参数```CBRK_NO_VISION```为0来使能视觉位置估计。 

> 设置系统参数```ATT_EXT_HDG_M```为1或者2来使能外部朝向估计。设置为1使用视觉，设置为2使用运动捕捉。

#### Disabling barometer fusion
If a highly accurate altitude is already available from vision or mocap information, it may be useful to disable the baro correction in LPE to reduce drift on the Z axis.

There is a bit field for this in the parameter `LPE_FUSION`, which you can set from QGroundControl. Just uncheck "fuse baro".

#### Tuning noise parameters

If your vision or mocap data is highly accurate, and you just want the estimator to track it tightly, you should reduce the standard deviation parameters, `LPE_VIS_XY` and `LPE_VIS_Z` (for vision) or `LPE_VIC_P` (for motion capture). Reducing them will cause the estimator to trust the incoming pose estimate more. You may need to set them lower than the allowed minimum and force-save.