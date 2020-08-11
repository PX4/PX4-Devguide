# Setting up a Developer Environment (Toolchain)

PX4 code can be developed on [Linux](../setup/dev_env_linux.md), [Mac OS](../setup/dev_env_mac.md), or [Windows](../setup/dev_env_windows.md). Recomendamos la [edición LTS de Ubuntu Linux](https://wiki.ubuntu.com/LTS), ya que esto permite compilar en [todos los sistemas compatibles con PX4](#supported-targets) y utilizando la mayoría de [simuladores](../simulation/README.md) y [ROS](../ros/README.md).

## Sistemas compatibles

La siguiente tabla muestra para qué objetivos PX4 se puede compilar en cada sistema operativo.

| Objetivo                                                                                                                                                                                                                                                                                         | Linux (Ubuntu) | Mac | Windows |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |:--------------:|:---:|:-------:|
| **NuttX based hardware:** [Pixhawk Series](https://docs.px4.io/master/en/flight_controller/pixhawk_series.html), [Crazyflie](https://docs.px4.io/master/en/flight_controller/crazyflie2.html), [Intel® Aero Ready to Fly Drone](https://docs.px4.io/master/en/flight_controller/intel_aero.html) |       X        |  X  |    X    |
| [Qualcomm Snapdragon Flight hardware](https://docs.px4.io/master/en/flight_controller/snapdragon_flight.html)                                                                                                                                                                                    |       X        |     |         |
| **Linux-based hardware:** [Raspberry Pi 2/3](https://docs.px4.io/master/en/flight_controller/raspberry_pi_navio2.html)                                                                                                                                                                           |       X        |     |         |
| **Simulación:** [jMAVSim SITL](../simulation/jmavsim.md)                                                                                                                                                                                                                                         |       X        |  X  |    X    |
| **Simulación:** [Gazebo SITL](../simulation/gazebo.md)                                                                                                                                                                                                                                           |       X        |  X  |         |
| **Simulación:** [ROS con Gazebo](../simulation/ros_interface.md)                                                                                                                                                                                                                                 |       X        |     |         |

## Entorno de desarrollo

La installación del entorno de desarrollo se trata a continuación:

- [Mac OS](../setup/dev_env_mac.md)
- [Linux](../setup/dev_env_linux.md)
- [Windows](../setup/dev_env_windows.md)

Si acostumbras a tratar con Docker también puedes usar uno de los contenedores preparados: [Docker Containers](../test_and_ci/docker.md)