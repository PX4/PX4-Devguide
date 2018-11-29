# Computer Vision (VIO, Avoidance)

[Computer vision](https://en.wikipedia.org/wiki/Computer_vision) techniques enable computers to use visual data to make sense of their environment. 

PX4 uses computer vision systems (primarily running on [Companion Computers](../companion_computer/pixhawk_companion.md)) in order to support the following features:
- [Optical Flow](https://docs.px4.io/en/sensor/optical_flow.html) uses a downward facing camera and a downward facing distance sensor for position estimation.
- [Visual Inertial Odometry](#vio) is used for navigation without a global position source (e.g. indoors).
- [Obstacle Avoidance](https://docs.px4.io/en/computer_vision/obstacle_avoidance.html) provides navigation around obstacles when flying a planned path (currently missions are supported). This uses [PX4/avoidance](https://github.com/PX4/avoidance) running on a companion computer.
- [Collision Prevention](https://docs.px4.io/en/computer_vision/collision_prevention.html) is used to stop vehicles before they can crash into an obstacle (primarily when flying in manual modes).


## Visual Inertial Odometry (VIO) {#vio}

VIO is used for estimating the *pose* (position and orientation) of a moving vehicle relative to a *local* starting position.
This can be used to navigate a vehicle in the absence of GPS (or another global position source), and is used, in particular, for indoor navigation.

The technique uses [Visual Odometry](https://en.wikipedia.org/wiki/Visual_odometry) to estimate vehicle *pose* from visual information, combined with inertial measurements from an IMU (to correct for errors associated with rapid vehicle movement resulting in poor image capture).

For information about VIO see:
- [External Position Estimation](../ros/external_position_estimation.md)
- [EKF > External Vision System](../tutorials/tuning_the_ecl_ekf.md#external-vision-system)
- [Snapdragon > Installation > Install Snap VIO](https://docs.px4.io/en/flight_controller/snapdragon_flight_software_installation.html#install-snap-vio)


## Optical Flow {#optical_flow}


For information about optical flow see:
- [Optical Flow](https://docs.px4.io/en/sensor/optical_flow.html)
  - [PX4Flow Smart Camera](https://docs.px4.io/en/sensor/px4flow.html)
- [EKF > Optical Flow](../tutorials/tuning_the_ecl_ekf.md#optical-flow)
