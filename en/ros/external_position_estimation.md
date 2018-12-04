# Using Vision or Motion Capture Systems

This page aims at getting a PX4 based system using position data from sources other than GPS (such as motion capture systems like VICON and Optitrack and vision based estimation systems like [ROVIO](https://github.com/ethz-asl/rovio), [SVO](https://github.com/uzh-rpg/rpg_svo) or [PTAM](https://github.com/ethz-asl/ethzasl_ptam)).

Position estimates can be sent both from an onboard computer as well as from offboard (example : VICON).
This data is used to update its local position estimate relative to the local origin. 
Heading from the vision/motion capture system can also be optionally integrated by the attitude estimator.

The system can then be used for applications such as position hold indoors or waypoint navigation based on vision.

For vision, the MAVLink message used to send the pose data is [VISION_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE) and the message for all motion capture systems is [ATT_POS_MOCAP](https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP) messages.

[ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY) can be used to share MoCap data with PX4. 
This can be done using the odom plugin of MAVROS.

The MAVROS ROS-MAVLink interface has default implementations to send these messages. 
They can also be sent using pure C/C++ code and direct use of the MAVLink library. 
The ROS topics are: `/mavros/mocap/pose` for MoCap systems and `/mavros/vision_pose/pose` for vision. 
Check [mavros_extras](http://wiki.ros.org/mavros_extras) for further info.

## EKF2 Tuning for External Position Estimate

A set of parameters need to be set in order for EKF2 to use the external position estimation. 
Adjust the parameters in *QGroundControl* (**Vehicle Setup > Parameters > EKF2**).

Parameter | Setting for Vision/MoCap Systems
--- | ---
[EKF2_AID_MASK](../advanced/parameter_reference.md#EKF2_AID_MASK) | Set *vision position fusion* and *vision yaw fusion*
[EKF2_HGT_MODE](../advanced/parameter_reference.md#EKF2_HGT_MODE) | Set to *Vision* to use the vision a primary source for altitude estimation.
[EKF2_EV_DELAY](../advanced/parameter_reference.md#EKF2_EV_DELAY) | Set to the difference between the timestamp of the measurement and the "actual" capture time. For more information see [below](#tuning-EKF2_EV_DELAY).
[EKF2_EV_POS_X](../advanced/parameter_reference.md#EKF2_EV_POS_X), [EKF2_EV_POS_Y](../advanced/parameter_reference.md#EKF2_EV_POS_Y), [EKF2_EV_POS_Z](../advanced/parameter_reference.md#EKF2_EV_POS_Z) | Set the position of the vision sensor (or MoCap markers) with respect to the robot's body frame.

> **Tip** Reboot the flight controller in order for parameter changes to take effect.

Now, you will need to feed the external position data to the flight controller using the [VISION_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE). 
MAVROS provides an easy interface to do this through the `vision_pose_estimate` plugin.


#### Tuning EKF2_EV_DELAY {#tuning-EKF2_EV_DELAY}

[EKF2_EV_DELAY](../advanced/parameter_reference.md#EKF2_EV_DELAY) is the *Vision Position Estimator delay relative to IMU measurements*.

Or in other words, it is the difference between the vision system timestamp and the "actual" capture time that would have been recorded by the IMU clock (the "base clock" for EKF2).

Technically this can be set to 0 if there is correct timestamping (not just arrival time) and timesync (e.g NTP) between MoCap and (for example) ROS computers. 
In reality, this needs some empirical tuning since delays in the entire MoCap->PX4 chain are very setup-specific. 
It is rare that a system is setup with an entirely synchronised chain!

A rough estimate of the delay can be obtained from logs by checking the offset between IMU rates and the EV rates:

![ekf2_ev_delay log](../../assets/ekf2/ekf2_ev_delay_tuning.png)

The value can further be tuned by varying the parameter to find the value that yields the lowest EKF innovations during dynamic maneuvers.

## LPE Tuning for Vision or MoCap

You will first need to [switching to the LPE estimator](advanced/switching_state_estimators.md) by setting the [SYS_MC_EST_GROUP](../advanced/parameter_reference.md#SYS_MC_EST_GROUP) parameter.

> **Note** If targeting `px4_fmu-v2` hardware you will need to use a firmware version that includes the LPE module (firmware for other FMU-series hardware includes both LPE and and EKF).
  The LPE version can be found in the zip file for each PX4 release or it can be built from source using the build command  `make px4_fmu-v2_lpe`.
  See [Building the Code](../setup/building_px4.md) for more details.

#### Enabling External Pose Input

A set of parameters need to be set in order for LPE to use the external pose input. 
Adjust the parameters in *QGroundControl* (**Vehicle Setup > Parameters > Local Position Estimator**).

Parameter | Setting for Vision/MoCap Systems
--- | ---
[LPE_FUSION](../advanced/parameter_reference.md#LPE_FUSION) | Vision integration is enabled if *fuse vision position* is checked (it is enabled by default).
[ATT_EXT_HDG_M](../advanced/parameter_reference.md#ATT_EXT_HDG_M) | Set to 1 or 2 to enable external heading integration. Setting it to 1 will cause vision to be used, while 2 enables MoCap heading use. 
 

#### Disabling Barometer Fusion

If a highly accurate altitude is already available from vision or MoCap information, it may be useful to disable the baro correction in LPE to reduce drift on the Z axis.

This can be done by in *QGroundControl* by unchecking the *fuse baro* option in the [LPE_FUSION](../advanced/parameter_reference.md#LPE_FUSION) parameter.

#### Tuning Noise Parameters

If your vision or MoCap data is highly accurate, and you just want the estimator to track it tightly, you should reduce the standard deviation parameters, [LPE_VIS_XY](../advanced/parameter_reference.md#LPE_VIS_XY) and [LPE_VIS_Z](../advanced/parameter_reference.md#LPE_VIS_Z) (for vision) or [LPE_VIC_P](../advanced/parameter_reference.md#LPE_VIC_P) (for motion capture). 
Reducing them will cause the estimator to trust the incoming pose estimate more. 
You may need to set them lower than the allowed minimum and force-save. 

> **Tip** If performance is still poor, try increasing the [LPE_PN_V](../advanced/parameter_reference.md#LPE_PN_V) parameter. This will cause the estimator to trust measurements more during velocity estimation.

## Asserting on Reference Frames

This section shows how to setup the system with the proper reference frames. 
There are various representations but we will use two of them: ENU and NED. 

* ENU has a ground-fixed frame where *x* axis points East, *y* points North and *z* up. 
  Robot frame is *x* towards the front, *z* up and *y* accordingly.
* NED has *x* towards North, *y* East and *z* down. 
  Robot frame is *x* towards the front, *z* down and *y* accordingly.

Frames are shown in the image below: NED on the left while ENU on the right.

![Reference frames](../../assets/lpe/ref_frames.png)

With the external heading estimation, however, magnetic North is ignored and faked with a vector corresponding to world *x* axis (which can be placed freely at MoCap calibration); 
yaw angle will be given with respect to local *x*.

> **Note** When creating the rigid body in the MoCap software, remember to first align the robot's local *x* axis with the world *x* axis otherwise yaw estimation will have an initial offset.

### Using MAVROS

With MAVROS this operation is straightforward. 
ROS uses ENU frames as convention, therefore position feedback must be provided in ENU. 
If you have an Optitrack system you can use [mocap_optitrack](https://github.com/ros-drivers/mocap_optitrack) node which streams the object pose on a ROS topic already in ENU. 
With a remapping you can directly publish it on `mocap_pose_estimate` as it is without any transformation and MAVROS will take care of NED conversions.

### Without MAVROS

If you do not use MAVROS or ROS in general, you need to stream data over MAVLink with [ATT_POS_MOCAP](../advanced/parameter_reference.md#ATT_POS_MOCAP) message. 
In this case you will need to apply a custom transformation depending on the system in order to obtain NED convention.

Let us take as an example the Optitrack framework; in this case the local frame has $$x$$ and $$z$$ on the horizontal plane (*x* front and *z* right) while *y* axis is vertical and pointing up. 
A simple trick is swapping axis in order to obtained NED convention. 

We call *x_{mav}*, *y_{mav}* and *z_{mav}* the coordinates that are sent through MAVLink as position feedback, then we obtain:

*x_{mav}* = *x_{mocap}*
*y_{mav}* = *z_{mocap}*
*z_{mav}* = - *y_{mocap}*

Regarding the orientation, keep the scalar part *w* of the quaternion the same and swap the vector part *x*, *y* and *z* in the same way. 
You can apply this trick with every system; 
you need to obtain a NED frame, look at your MoCap output and swap axis accordingly.

## Specific System Setups

### OptiTrack MoCap

The following steps explain how to feed position estimates from an [OptiTrack](http://optitrack.com/systems/#robotics) system to PX4. 
It is assumed that the MoCap system is calibrated. 
See [this video](https://www.youtube.com/watch?v=cNZaFEghTBU) for a tutorial on the calibration process.

#### Steps on the *Motive* MoCap software

* Align your robot's forward direction with the the [system +x-axis](https://v20.wiki.optitrack.com/index.php?title=Template:Coordinate_System)
* [Define a rigid body in the Motive software](https://www.youtube.com/watch?v=1e6Qqxqe-k0). Give the robot a name that does not contain spaces, e.g. `robot1` instead of `Rigidbody 1`
* [Enable Frame Broadacst and VRPN streaming](https://www.youtube.com/watch?v=yYRNG58zPFo)
* Set the Up axis to be the Z axis (the default is Y)

#### Getting pose data into ROS

* Install the `vrpn_client_ros` package
* You can get each rigid body pose on an individual topic by running
  ```bash
  roslaunch vrpn_client_ros sample.launch server:=<mocap machine ip>
  ```
  
If you named the rigidbody as `robot1`, you will get a topic like `/vrpn_client_node/robot1/pose`

#### Relaying pose data to PX4

MAVROS provides a plugin to relay pose data published on `/mavros/vision_pose/pose` to PX4. 
Assuming that MAVROS is running, you just need to **remap** the pose topic that you get from MoCap `/vrpn_client_node/<rigid_body_name>/pose` directly to `/mavros/vision_pose/pose`. 
Note that there is also a `mocap` topic that MAVROS provides to feed `ATT_POS_MOCAP` to PX4, but it is not applicable for EKF2. However, it is applicable with LPE.

Assuming that you have configured EKF2 parameters as described above, PX4 now is set and fusing MoCap data.

You are now set to proceed to the first flight.


## First Flight

At this point, if you followed those steps, you are ready to test your setup. 

Be sure to perform the following checks:

* **Before** creating the rigid body, align the robot with world x axis
* Stream over MAVLink and check the MAVLink inspector with *QGroundControl*, the local pose topic should be in NED
* Move the robot around by hand and see if the estimated local position is consistent (always in NED)
* Rotate the robot on the vertical axis and check the yaw with the MAVLink inspector

If those steps are consistent, you can try your first flight.

Put the robot on the ground and start streaming MoCap feedback. 
Lower your left (throttle) stick and arm the motors.

At this point, with the left stick at the lowest position, switch to position control. 
You should have a green light. 
The green light tells you that position feedback is available and position control is now activated. 

Put your left stick at the middle, this is the dead zone. 
With this stick value, the robot maintains its altitude; 
raising the stick will increase the reference altitude while lowering the value will decrease it. 
Same for right stick on x and y. 

Increase the value of the left stick and the robot will take off, 
put it back to the middle right after. Check if it is able to keep its position.

If it works, you may want to set up an [offboard](offboard_control.md) experiment by sending position-setpoint from a remote ground station.
