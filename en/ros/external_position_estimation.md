[](assets/lpe)# Using Vision or Motion Capture systems

> **Info** Before following the instructions below, ensure that your autopilot has a firmware version with the LPE modules enabled. The LPE version of the PX4 firmware can be found inside the zip file of the latest PX4 release or it can be built from source using a build command such as `make px4fmu-v2_lpe`. See [Building the Code](../setup/building_px4.md) for more details.

This page aims at getting a PX4 based system using position data from sources other than GPS (such as motion capture systems like VICON and Optitrack and vision based estimation systems like [ROVIO](https://github.com/ethz-asl/rovio), [SVO](https://github.com/uzh-rpg/rpg_svo) or [PTAM](https://github.com/ethz-asl/ethzasl_ptam) )

Position estimates can be sent both from an onboard computer as well as from offboard (example : VICON).  This data is used to update its local position estimate relative to the local origin. Heading from the vision/motion capture system can also be optionally integrated by the attitude estimator.

The system can then be used for applications such as position hold indoors or waypoint navigation based on vision.

For vision, the mavlink message used to send the pose data is [VISION_POSITION_ESTIMATE](http://mavlink.org/messages/common#VISION_POSITION_ESTIMATE) and the message for all motion capture systems is [ATT_POS_MOCAP](http://mavlink.org/messages/common#ATT_POS_MOCAP) messages.

The mavros ROS-Mavlink interface has default implementations to send these messages. They can also be sent using pure C/C++ code and direct use of the MAVLink library. The ROS topics are: `mocap_pose_estimate` for mocap systems and `vision_pose_estimate` for vision. Check [mavros_extras](http://wiki.ros.org/mavros_extras) for further info.

**This feature has only been tested to work with the LPE estimator.**

## LPE Tuning for Vision or Mocap

### Enabling external pose input
You need to set a few parameters (from QGroundControl or the NSH shell) to enable or disable vision/mocap usage in the system.

Set the system parameter `ATT_EXT_HDG_M` to 1 or 2 to enable external heading integration. Setting it to 1 will cause vision to be used, while 2 enables mocap heading use.

Vision integration is enabled by default in LPE. You can control this using the`LPE_FUSION` parameter in QGroundControl. Make sure that "fuse vision position" is checked.

#### Disabling barometer fusion
If a highly accurate altitude is already available from vision or mocap information, it may be useful to disable the baro correction in LPE to reduce drift on the Z axis.

There is a bit field for this in the parameter `LPE_FUSION`, which you can set from QGroundControl. Just uncheck "fuse baro".

#### Tuning noise parameters

If your vision or mocap data is highly accurate, and you just want the estimator to track it tightly, you should reduce the standard deviation parameters, `LPE_VIS_XY` and `LPE_VIS_Z` (for vision) or `LPE_VIC_P` (for motion capture). Reducing them will cause the estimator to trust the incoming pose estimate more. You may need to set them lower than the allowed minimum and force-save. 

> **Tip** If performances are still bad, try to increase `LPE_PN_V` parameter in order to trust measurement more regarding velocity estimation.

## Asserting on reference frames
This section shows how to setup the system with the proper reference frames. There are various representations but we will use two of them: ENU and NED. 

* ENU representation has a ground-fixed frame where $$x$$ axis points East, $$y$$ points North and $$z$$ up. Robot frame is $$x$$ towards the front, $$z$$ up and $$y$$ accordingly.

* NED has $$x$$ towards North, $$y$$ East and $$z$$ down. Robot frame is $$x$$ towards the front, $$s$$ down and $$y$$ accordingly.

Frames are shown in the image below: NED on the left while ENU on the right.
![Reference frames](../../assets/lpe/ref_frames.png)

With the external heading estimation, however, magnetic North is ignored and faked with a vector corresponding to world $$x$$ axis (which can be placed freely at mocap calibration), yaw angle will be given respect to local $$x$$.

###Using Mavros

With MAVROS this operation is straightforward. ROS uses ENU frames as convention, therefore position feedback must be provided in ENU. If you have an Optitrack system you can use 














