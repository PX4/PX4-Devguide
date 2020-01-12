# 硬件仿真

对于四旋翼，硬件仿真（SIH）是[硬件在环仿真（HITL）](../simulation/hitl.md)的替代品。 在这个设置中，所有的数据处理工作都在嵌入式硬件（PIXHAWK）中完成，包括控制器、状态估计器和仿真器。 与PIXHAWK连接的电脑只用来显示虚拟的载具。

![仿真器 MAVLink API](../../assets/diagrams/SIH_diagram.png)

与硬件在环仿真相比，硬件仿真有以下两点好处：

- It ensures synchronous timing by avoiding the bidirectional connection to the computer. As a result the user does not need such a powerful desktop computer.

- The whole simulation remains inside the PX4 environment. Developers who are familiar with PX4 can more easily incorporate their own mathematical model into the simulator. They can, for instance, modify the aerodynamic model, or noise level of the sensors, or even add a sensor to be simulated.

The SIH can be used by new PX4 users to get familiar with PX4 and the different modes and features, and of course to learn to fly a quadrotor with the real RC controller.

The dynamic model is described in this [pdf report](https://github.com/PX4/Devguide/raw/master/assets/simulation/SIH_dynamic_model.pdf).

Furthermore, the physical parameters representing the vehicle (such as mass, inertia, and maximum thrust force) can easily be modified from the [SIH parameters](../advanced/parameter_reference.md#simulation-in-hardware).

## Requirements

为了运行硬件仿真，你需要一个飞控板硬件（例如：Pixhawk飞控）。 如果你计划使用一对[无线电控制发射机和接收机](https://docs.px4.io/en/getting_started/rc_transmitter_receiver.html)，你也应该安装好。 或者，使用*QGC地面站*、[操纵杆](https://docs.qgroundcontrol.com/en/SetupView/Joystick.html)也能被用来仿真一个无线电控制系统。

硬件仿真可以在除了FMUv2硬件之外的所有Pixhawk飞控板上使用。 它可以在固件主分支和发布版本v1.9.0及以上版本中使用。

## 配置硬件仿真（SIH）

运行SIH和挑选一个机架一样简单。 将飞控和电脑用USB线连接起来，让它通电启动，然后使用地面站选择[SIH 机架](../airframes/airframe_reference.md#simulation-copter) 飞控接下来会重启

当选择了SIH 机架之后，SIH模块就自动启动了，车辆应该显示在地面站的地图上

## 设置显示

The simulated quadrotor can be displayed in jMAVSim from PX4 v1.11.

1. Close *QGroundControl* (if opened).
2. Unplug and replug the hardware autopilot (allow a few seconds for it to boot).
3. Start jMAVSim by calling the script **jmavsim_run.sh** from a terminal: ```./Tools/jmavsim_run.sh -q -d /dev/ttyACM0 -b 921600 -r 250 -o``` where the flags are 
    - `-q` to allow the communication to *QGroundControl* (optional).
    - `-d` to start the serial device `/dev/ttyACM0` on Linux. On macOS this would be `/dev/tty.usbmodem1`.
    - `-b` to set the serial baud rate to `921600`.
    - `-r` to set the refresh rate to `250` Hz (optional).
    - `-o` to start jMAVSim in *display Only* mode (i.e. the physical engine is turned off and jMAVSim only displays the trajectory given by the SIH in real-time).
4. After few seconds, *QGroundControl* can be opened again.

At this point, the system can be armed and flown. The vehicle can be observed moving in jMAVSim, and on the QGC **Fly** view.

## 鸣谢

The SIH was developed by Coriolis g Corporation, a Canadian company developing a new type of Vertical Takeoff and Landing (VTOL) Unmanned Aerial Vehicles (UAV) based on passive coupling systems.

Specialized in dynamics, control, and real-time simulation, they provide the SIH as a simple simulator for quadrotors released for free under BSD license.

Discover their current platform at [www.vogi-vtol.com](http://www.vogi-vtol.com/).