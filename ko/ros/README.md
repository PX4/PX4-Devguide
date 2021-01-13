!REDIRECT "https://docs.px4.io/master/ko/ros/"

# ROS를 활용한 로보틱스

[ROS](http://www.ros.org/) (Robot Operating System)는 로보틱스용 범용 라이브러리이며, PX4에 [offboard control](../ros/mavros_offboard.md)용으로 사용할 수 있습니다. [MAVROS](../ros/mavros_installation.md) 노드를 사용하여 하드웨어상에서 동작하거나 [Gazebo Simulator](../simulation/ros_interface.md)를 사용하는 PX4와 통신할 수 있습니다.

이 절에서는 PX4를 가지고 offboard control을 하기 위해 ROS를 사용하는 주제를 다룹니다.

> **Tip** ROS는 리눅스 플랫폼만을 공식 지원합니다.

## Installation

The easiest way to setup PX4 simulation with ROS (on Ubuntu Linux) is to use the standard installation script that can be found at [Development Environment on Linux > Gazebo with ROS](../setup/dev_env_linux_ubuntu.md#rosgazebo). The script installs everything you need: PX4, ROS, the Gazebo simulator, and [MAVROS](../ros/mavros_installation.md).

> **Note** If you just need to install ROS then follow the [ROS Melodic installation instructions](http://wiki.ros.org/melodic/Installation) for your platform.

## External Resources

- [XTDrone](https://github.com/robin-shaun/XTDrone/blob/master/README.en.md) - 컴퓨터 비전용 ROS + PX4 시뮬레이션 환경. 시작하기에 필요한 모든 내용은 [XTDrone Manual](https://www.yuque.com/xtdrone/manual_en)에 있습니다!