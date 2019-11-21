# 硬件仿真

对于四旋翼，硬件仿真是硬件在环仿真的替代品。 在这个设置中，所有的数据处理工作都在嵌入式硬件（PIXHAWK）中完成，包括控制器、状态估计器和仿真器。 与PIXHAWK连接的电脑只用来显示虚拟的载具。

![Simulator MAVLink API](../../assets/diagrams/SIH_diagram.png)

与硬件在环仿真相比，硬件仿真有以下两点好处：

- It ensures synchronous timing by avoiding the bidirectional connection to the computer. As a result the user does not need such a powerful desktop computer.

- The whole simulation remains inside the PX4 environment. Developers who are familiar with PX4 can more easily incorporate their own mathematical model into the simulator. They can, for instance, modify the aerodynamic model, or noise level of the sensors, or even add a sensor to be simulated.

The SIH can be used by new PX4 users to get familiar with PX4 and the different modes and features, and of course to learn to fly a quadrotor with the real RC controller.

The dynamic model is described in this [pdf report](https://github.com/PX4/Devguide/raw/master/assets/simulation/SIH_dynamic_model.pdf).

Furthermore, the physical parameters representing the vehicle (such as mass, inertia, and maximum thrust force) can easily be modified from the [SIH parameters](../advanced/parameter_reference.md#simulation-in-hardware).

## Requirements

To run the SIH, you will need a [flight controller hardware](https://docs.px4.io/en/flight_controller/) (e.g. a Pixhawk-series board). If you are planning to use a [radio control transmitter and receiver pair](https://docs.px4.io/en/getting_started/rc_transmitter_receiver.html) you should have that too. Alternatively, using *QGroundControl*, a [joystick](https://docs.qgroundcontrol.com/en/SetupView/Joystick.html) can be used to emulate a radio control system.

The SIH is compatible with all Pixhawk-series boards except those based on FMUv2. It is available on the firmware master branch and release versions v1.9.0 and above.

## Setting up SIH

Running the SIH is as easy as selecting an airframe. Plug the autopilot to the desktop computer with a USB cable, let it boot, then using a ground control station select the [SIH airframe](../airframes/airframe_reference.md#simulation-copter). The autopilot will then reboot.

When the SIH airframe is selected, the SIH module starts by itself, the vehicle should be displayed on the ground control station map.

## Setting up the Display

The simulated quadrotor can be displayed in jMAVSim (coming soon).

## Credits

The SIH was developed by Coriolis g Corporation, a Canadian company developing a new type of Vertical Takeoff and Landing (VTOL) Unmanned Aerial Vehicles (UAV) based on passive coupling systems.

Specialized in dynamics, control, and real-time simulation, they provide the SIH as a simple simulator for quadrotors released for free under BSD license.

Discover their current platform at [www.vogi-vtol.com](http://www.vogi-vtol.com/).