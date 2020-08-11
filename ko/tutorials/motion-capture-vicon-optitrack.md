# 움직임 촬영(Motion Capture)기술을 활용한 비행 (VICON, Optitrack)

> **Warning** **작성중입니다**. 이 주제에서는 [외부 위치 추정(ROS)](../ros/external_position_estimation.md)과 약간 겹칩니다.

Indoor motion capture systems like VICON and Optitrack can be used to provide position and attitude data for vehicle state estimation, orto serve as ground-truth for analysis. The motion capture data can be used to update PX4's local position estimate relative to the local origin. Heading (yaw) from the motion capture system can also be optionally integrated by the attitude estimator.

Pose (position and orientation) data from the motion capture system is sent to the autopilot over MAVLink, using the [ATT_POS_MOCAP](https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP) message. See the section below on coordinate frames for data representation conventions. The [mavros](../ros/mavros_installation.md) ROS-Mavlink interface has a default plugin to send this message. They can also be sent using pure C/C++ code and direct use of the MAVLink library.

## 처리 구조

It is **highly recommended** that you send motion capture data via an **onboard** computer (e.g Raspberry Pi, ODroid, etc.) for reliable communications. The onboard computer can be connected to the motion capture computer through WiFi, which offers reliable, high-bandwidth connection.

Most standard telemetry links like 3DR/SiK radios are **not** suitable for high-bandwidth motion capture applications.

## 좌표 틀

이 절에서는 적당한 참조 프레임 시스템을 설정하는 방법을 알려드리겠습니다. 다양한 표현법이 있지만 ENU와 NED 방식을 활용하겠습니다.

* ENU는 지상 고정 좌표로서, **X**축은 동쪽, **Y** 축은 북쪽, **Z**축은 상공을 향합니다. 로봇/기체 틀 기준으로는 **X**축은 전면, **Z**축은 상단, **Y**축은 좌측을 향합니다.
* NED에서 **X**축은 북쪽, **Y**축은 동쪽, **Z**축은 지면을 향합니다. The robot/vehicle body frame has **X** towards the front, **Z** down and **Y** accordingly.

Frames are shown in the image below. NED on the left, ENU on the right: ![Reference frames](../../assets/lpe/ref_frames.png)

With the external heading estimation, however, magnetic North is ignored and faked with a vector corresponding to world *x* axis (which can be placed freely at mocap calibration); yaw angle will be given respect to local *x*.

> **Warning** When creating the rigid body in the motion capture software, remember to first align the robot with the world **X** axis otherwise yaw estimation will have an initial offset.

## Estimator choice

### LPE and Attitude Estimator Q

### EKF2

The ROS topic for motion cap `mocap_pose_estimate` for mocap systems and `vision_pose_estimate` for vision. Check [mavros_extras](http://wiki.ros.org/mavros_extras) for further info.

## Testing

## Troubleshooting