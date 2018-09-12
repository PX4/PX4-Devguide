# 파일 및 코드 설치

PX4 코드는 [Linux](../setup/dev_env_linux.md) 또는 [Mac OS](../setup/dev_env_mac.md)에서 개발할 수 있습니다. [Ubuntu Linux LTS](https://wiki.ubuntu.com/LTS) 에디션을 권장합니다. [모든 PX4 타겟](#supported-targets)을 구축하고 대부분의 [시뮬레이터](../simulation/README.md)와 [ROS](../ros/README.md)를 사용할 수 있기 때문입니다.

## Supported Targets

The table below show what PX targets you can build on each OS.

| Target                                                                                                                                                                                                                                                                      | Linux (Ubuntu) | Mac | Windows |
| --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |:--------------:|:---:|:-------:|
| **NuttX based hardware:** [Pixhawk Series](https://docs.px4.io/en/flight_controller/pixhawk_series.html), [Crazyflie](https://docs.px4.io/en/flight_controller/crazyflie2.html), [Intel® Aero Ready to Fly Drone](https://docs.px4.io/en/flight_controller/intel_aero.html) |       X        |  X  |    X    |
| [Qualcomm Snapdragon Flight hardware](https://docs.px4.io/en/flight_controller/snapdragon_flight.html)                                                                                                                                                                      |       X        |     |         |
| **Linux-based hardware:** [Raspberry Pi 2/3](https://docs.px4.io/en/flight_controller/raspberry_pi_navio2.html), [Parrot Bebop](https://docs.px4.io/en/flight_controller/bebop.html)                                                                                        |       X        |     |         |
| **Simulation:** [jMAVSim SITL](../simulation/jmavsim.md)                                                                                                                                                                                                                    |       X        |  X  |    X    |
| **Simulation:** [Gazebo SITL](../simulation/gazebo.md)                                                                                                                                                                                                                      |       X        |  X  |         |
| **Simulation:** [ROS with Gazebo](../simulation/ros_interface.md)                                                                                                                                                                                                           |       X        |     |         |

## Development Environment

The installation of the development environment is covered below:

- [Mac OS](../setup/dev_env_mac.md)
- [Linux](../setup/dev_env_linux.md)
- [Windows](../setup/dev_env_windows.md)

If you're familiar with Docker you can also use one of the prepared containers: [Docker Containers](../test_and_ci/docker.md)