# Using Vision or Motion Capture Systems

> **Info** Before following the instructions below, ensure that your autopilot has a firmware version with the LPE modules enabled. The LPE version of the PX4 firmware can be found inside the zip file of the latest PX4 release or it can be built from source using a build command such as `make px4fmu-v2_lpe`. See [Building the Code](../setup/building_px4.md) for more details.

This page aims at getting a PX4 based system using position data from sources other than GPS (such as motion capture systems like VICON and Optitrack and vision based estimation systems like [ROVIO](https://github.com/ethz-asl/rovio), [SVO](https://github.com/uzh-rpg/rpg_svo) or [PTAM](https://github.com/ethz-asl/ethzasl_ptam) )

Position estimates can be sent both from an onboard computer as well as from offboard (example : VICON).  This data is used to update its local position estimate relative to the local origin. Heading from the vision/motion capture system can also be optionally integrated by the attitude estimator.

The system can then be used for applications such as position hold indoors or waypoint navigation based on vision.

For vision, the MAVLink message used to send the pose data is [VISION_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE) and the message for all motion capture systems is [ATT_POS_MOCAP](https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP) messages.

The mavros ROS-MAVLink interface has default implementations to send these messages. They can also be sent using pure C/C++ code and direct use of the MAVLink library. The ROS topics are: `/mavros/mocap/pose` for mocap systems and `/mavros/vision_pose/pose` for vision. Check [mavros_extras](http://wiki.ros.org/mavros_extras) for further info.

## EKF2 Tuning for External Position Estimate

A set of paramters need to be set in order for EKF2 to use the external position estimation. You can use QGroundContorl (QGC) to easily adjust these paramters. All of the following paramters can be found in the EKF2 tab in QGC.

* External position estimate can be enabled by setting the `EKF_AID_MASK` to enable vision position and yaw fusion
* To use the external height estimate for altitude estimation, set `EKF2_HGT_MODE` to use vision
* Adjust the `EKF2_EV_DELAY` paramter. This value actually represents how far off the timestamp of the measurement is off from the "actual" time it was captured at. It can technically be set to 0 if there is correct timestamping (not just arrival time) and timesync (e.g NTP) between mocap and ROS computers. In reality, this needs some empirical tuning since delays in the entire Mocap->PX4 chain are very setup specific and there is rarely a well setup system with an entirely synchronised chain.

* Use `EKF2_EV_POS_X`, `EKF2_EV_POS_Y`, `EKF2_EV_POS_Z` to set the position of the vision sensor (or mocap markers) with respect to the robot's body frame.
* Reboot the flight controller in order for the paramters to take effect.

Now, you will need to feed the external position data to the flight controller using the [VISION_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE). MAVROS provides an easy interface to do this through the `vision_pose_estimate` plugin.


## LPE Tuning for Vision or Mocap

### Enabling external pose input

You need to set a few parameters (from QGroundControl or the NSH shell) to enable or disable vision/mocap usage in the system.

Set the system parameter `ATT_EXT_HDG_M` to 1 or 2 to enable external heading integration. Setting it to 1 will cause vision to be used, while 2 enables mocap heading use. 

Vision integration is enabled by default in LPE. You can control this using the`LPE_FUSION` parameter in QGroundControl. Make sure that "fuse vision position" is checked.

#### Disabling barometer fusion

If a highly accurate altitude is already available from vision or mocap information, it may be useful to disable the baro correction in LPE to reduce drift on the Z axis.

There is a bit field for this in the parameter `LPE_FUSION`, which you can set from QGroundControl. Just uncheck "fuse baro".

#### Tuning Noise Parameters

If your vision or mocap data is highly accurate, and you just want the estimator to track it tightly, you should reduce the standard deviation parameters, `LPE_VIS_XY` and `LPE_VIS_Z` (for vision) or `LPE_VIC_P` (for motion capture). Reducing them will cause the estimator to trust the incoming pose estimate more. You may need to set them lower than the allowed minimum and force-save. 

> **Tip**If performance is still poor, try increasing the `LPE_PN_V` parameter. This will cause the estimator to trust measurements more during velocity estimation..

## Asserting on Reference Frames

This section shows how to setup the system with the proper reference frames. There are various representations but we will use two of them: ENU and NED. 

* ENU has a ground-fixed frame where *x* axis points East, *y* points North and *z* up. Robot frame is *x* towards the front, *z* up and *y* accordingly.

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

*x_{mav}* = *x_{mocap}*
*y_{mav}* = *z_{mocap}*
*z_{mav}* = - *y_{mocap}*

Regarding the orientation, keep the scalar part *w* of the quaternion the same and swap the vector part *x*, *y* and *z* in the same way. You can apply this trick with every system; you need to obtain a NED frame, look at your mocap output and swap axis accordingly.

## Specific System Setups

### OptiTrack MOCAP

The following steps explain how to feed position estimates from an [OptiTrack](http://optitrack.com/systems/#robotics) system to PX4. It is assumed that the MOCAP system is calibrated. See [this video](https://www.youtube.com/watch?v=cNZaFEghTBU) for a tutorial on the calibration process.

**Steps to do on the MOCAP software, Motive**

* Align your robot's forward direction with the the [system +x-axis](https://v20.wiki.optitrack.com/index.php?title=Template:Coordinate_System)
* [Define a rigid body in the Motive software](https://www.youtube.com/watch?v=1e6Qqxqe-k0). Give the robot a name that does not contain spaces, e.g. `robot1` instead of `Rigidbody 1`
* [Enable Frame Broadacst and VRPN streaming](https://www.youtube.com/watch?v=yYRNG58zPFo)
* Set the Up axis to be the Z axis (the default is Y)

**Getting pose data into ROS**

* Install the `vrpn_client_ros` package
* You can get each rigid body pose on an individual topic by running
```bash
roslaunch vrpn_client_ros sample.launch server:=<mocap machine ip>
```
If you named the rigidbody as `robot1`, you will get a topic like `/vrpn_client_node/robot1/pose`

**Relaying pose data to PX4**

MAVROS provies a plugin to relay pose data published on `/mavros/vision_pose/pose` to PX4. Assuming that MAVROS is running, you just need to **remap** the pose topic that you get from MOCAP `/vrpn_client_node/<rigid_body_name>/pose` directly to `/mavros/vision_pose/pose`.

Assuming that you have configured EKF2 paramters as described above, PX4 now is set and fusing MOCAP data.

You are now set to proceed to the first flight.


## First Flight

At this point, if you followed those steps, you are ready to test your setup. 

Be sure to perform the following checks:

* **Before** creating the rigid body, align the robot with world x axis
* Stream over MAVLink and check the MAVLink inspector with QGroundControl, the local pose topic should be in NED
* Move the robot around by hand and see if the estimated local position is consistent (always in NED)
* Rotate the robot on the vertical axis and check the yaw with the MAVLink inspector

If those steps are consistent, you can try your first flight.

Put the robot on the ground and start streaming mocap feedback. Lower your left (throttle) stick and arm the motors.

At this point, with the left stick at the lowest position, switch to position control. 
You should have a green light. The green light tells you that position feedback is available and position control is now activated. 

Put your left stick at the middle, this is the dead zone. 
With this stick value, the robot maintains its altitude; 
raising the stick will increase the reference altitude while lowering the value will decrease it. 
Same for right stick on x and y. 

Increase the value of the left stick and the robot will take off, 
put it back to the middle right after. Check if it is able to keep its position.

If it works, you may want to set up an [offboard](offboard_control.md) experiment by sending position-setpoint from a remote ground station.
