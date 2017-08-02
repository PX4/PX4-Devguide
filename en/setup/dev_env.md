# Installing Files and Code

PX4 code can be developed on [Linux](../setup/dev_env_linux.md) or [Mac OS](../setup/dev_env_mac.md). We recommend [Ubuntu Linux LTS edition](https://wiki.ubuntu.com/LTS) as this enables building [all PX4 targets](#supported-targets).

> **Warning** A [Windows](../setup/dev_env_windows.md) toolchain also exists but is not officially supported (we highly discourage its use). It is possible to build PX4 on Windows using a virtual machine running Ubuntu Linux, but this may not provide a reliable platform for Simulation. Before starting to develop on Windows, consider installing a dual-boot environment with [Ubuntu](http://ubuntu.com). 

## Supported Targets

The table below show what PX targets you can build on each OS.

Target | Linux (Ubuntu) | Mac | Windows
--|:--:|:--:|:--:
**NuttX based hardware:** [Pixhawk](../flight_controller/pixhawk.md), [Pixfalcon](../flight_controller/pixfalcon.md), [Pixracer](../flight_controller/pixracer.md), [Pixhawk 3 Pro](../flight_controller/pixhawk3_pro.md), [Crazyflie](../flight_controller/crazyflie2.md), [Intel® Aero Ready to Fly Drone](../flight_controller/intel_aero.md) | X | X | X
[Qualcomm Snapdragon Flight hardware](../flight_controller/snapdragon_flight.md) | X | | 
**Linux-based hardware:** [Raspberry Pi 2/3](../flight_controller/raspberry_pi_navio2.md), Parrot Bebop  | X | | 
**Simulation:** [jMAVSim SITL](../simulation/jmavsim.md) | X | X | X
**Simulation:** [Gazebo SITL](../simulation/gazebo.md) | X | X | 

## Development Environment

The installation of the development environment is covered below:

  * [Mac OS](../setup/dev_env_mac.md)
  * [Linux](../setup/dev_env_linux.md)
  * [Windows](../setup/dev_env_windows.md) (not recommended!)

If you're familiar with Docker you can also use one of the prepared containers: [Docker Containers](../test_and_ci/docker.md)