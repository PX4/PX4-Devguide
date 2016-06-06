# Using Vision or Motion Capture systems

This page aims at getting a PX4 based system using position data from sources other than GPS (such as motion capture systems like VICON and Optitrack and vision based estimation systems like [ROVIO](https://github.com/ethz-asl/rovio), [SVO](https://github.com/uzh-rpg/rpg_svo) or [PTAM](https://github.com/ethz-asl/ethzasl_ptam) )

Position estimates can be sent both from an onboard computer as well as from offboard (example : VICON).  This data is used to update its local position estimate relative to the local origin. Heading from the vision/motion capture system can also be optionally integrated by the attitude estimator. 

The system can then be used for applications such as position hold indoors or waypoint navigation based on vision.

For vision, the mavlink message used to send the pose data is [VISION_POSITION_ESTIMATE](http://mavlink.org/messages/common#VISION_POSITION_ESTIMATE) and the message for all motion capture systems is [ATT_POS_MOCAP](http://mavlink.org/messages/common#ATT_POS_MOCAP) messages. 

The mavros ROS-Mavlink interface has default implementations to send these messages. They can also be sent using pure C/C++ code and direct use of the MAVLink() library.

## Enabling external pose input
You need to set 2 parameters (from QGroundControl or the NSH shell) to enable or disable vision/mocap usage in the system.

<aside class="note">
Set the system parameter ```CBRK_NO_VISION``` to 0 to enable vision position integration. 
</aside>

<aside class="note">
Set the system parameter ```ATT_EXT_HDG_M``` to 1 or 2 to enable external heading integration. Setting it to 1 will cause vision to be used, while 2 enables mocap heading use.
</aside>
