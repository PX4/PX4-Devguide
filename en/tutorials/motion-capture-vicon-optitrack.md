# Flying with Motion Capture (VICON, Optitrack)

Indoor motion capture systems like VICON and Optitrack can be used to provide position and attitude data for vehicle state estimation, or to serve as ground-truth for analysis. 

It is **highly recommended** to send motion capture data from an onboard computer for reliable communications. Most standard telemetry links like 3DR/SiK radios are **not** suitable for high-bandwidth motion capture applications.

The motion capture data can be used to update PX4's local position estimate relative to the local origin. Heading (yaw) from the motion capture system can also be optionally integrated by the attitude estimator.

Pose (position and orientation) data from the motion capture system is sent over MAVLinkm using the [ATT_POS_MOCAP](http://mavlink.org/messages/common#ATT_POS_MOCAP) message. Note that this data is expected to be in the NED (North-East-Down) frame .

The [mavros ROS-Mavlink]() interface has default implementations to send this messages. They can also be sent using pure C/C++ code and direct use of the MAVLink library.