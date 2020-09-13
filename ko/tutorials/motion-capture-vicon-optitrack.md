# 움직임 촬영(Motion Capture)기술을 활용한 비행 (VICON, Optitrack)

> **Warning** **작성중입니다**. 이 주제에서는 [외부 위치 추정(ROS)](../ros/external_position_estimation.md)과 약간 겹칩니다.

Indoor motion capture systems like VICON and Optitrack can be used to provide position and attitude data for vehicle state estimation, orto serve as ground-truth for analysis. The motion capture data can be used to update PX4's local position estimate relative to the local origin. Heading (yaw) from the motion capture system can also be optionally integrated by the attitude estimator.

Pose (position and orientation) data from the motion capture system is sent to the autopilot over MAVLink, using the [ATT_POS_MOCAP](https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP) message. See the section below on coordinate frames for data representation conventions. The [mavros](../ros/mavros_installation.md) ROS-Mavlink interface has a default plugin to send this message. They can also be sent using pure C/C++ code and direct use of the MAVLink library.

## 처리 구조

온전한 통신의 수행을 위해 **내장 컴퓨터**(예: 라즈베리 파이, 오드로이드 등)로 동영상 촬영 데이터를 보내시는 방안을 **강력 추천**합니다. The onboard computer can be connected to the motion capture computer through WiFi, which offers reliable, high-bandwidth connection.

Most standard telemetry links like 3DR/SiK radios are **not** suitable for high-bandwidth motion capture applications.

## 좌표 틀

이 절에서는 적당한 참조 프레임 시스템을 설정하는 방법을 알려드리겠습니다. 다양한 표현법이 있지만 ENU와 NED 방식을 활용하겠습니다.

* ENU는 지상 고정 좌표로서, **X**축은 동쪽, **Y** 축은 북쪽, **Z**축은 상공을 향합니다. 로봇/기체 틀 기준으로는 **X**축은 전면, **Z**축은 상단, **Y**축은 좌측을 향합니다.
* NED에서 **X**축은 북쪽, **Y**축은 동쪽, **Z**축은 지면을 향합니다. 로봇/기체 틀 기준으로는 **X**축은 북쪽, **Y**축은 동쪽 **Z**축은 지면을 향합니다. 

아래 그림에서 프레임의 방향 상태를 보여드립니다. NED는 좌측, ENU는 우측에 있습니다: ![Reference frames](../../assets/lpe/ref_frames.png)

그러나 외부 방향 추정시, 자북은 무시하고 가상 세계 *X* 좌표 축을 따라 벡터 기준을 삼습니다(움직임 촬영 기법으로 보정할 때 언제든 자유롭게 둘 수 있습니다), 방위각면이 로컬 *x* 좌표를 두는 면입니다.

> **Warning** 움직임을 촬영하는 프로그램에서 강체를 만들 때, 세계 **X** 좌표 축에 로봇을 우선 맞춰야 함을 기억하십시오. 그렇지 않으면 방위 회전각 추정시 초기 오프셋으로 처리합니다.

## 추정자 선택

### LPE와 고도 추정자 Q

### EKF2

움직임 촬영 기법에 있어 mocap 시스템에선 `mocap_pose_estimate`을 시각 정보 처리에는 `vision_pose_estimate`를 ROS 토픽 메시지로 다룹니다. 더 자세한 정보는 [mavros_extras](http://wiki.ros.org/mavros_extras)를 확인하십시오.

## 시험

## 문제 해결