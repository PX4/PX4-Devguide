# jMAVSim 进行 SITL 仿真

jMAVSim是一个简单的多旋翼/四旋翼仿真软件，它可以允许你在仿真环境中飞行运行着 PX4 的 *旋翼* 无人机。 它很容易设置，可以用来测试您的工具是否可以起飞、飞行、降落、并对各种故障条件 (例如 gps 故障) 做出适当的反应。

<strong>支持机型：</strong>

* 四旋翼

本问主要演示如何设置 jMAVSim 以连接到 SITL 版本的 PX4 。

> **Tip** jMAVSim 也可以用来进行 HITL 仿真 ([看这里](../simulation/hitl.md#using-jmavsim-quadrotor)).

## 仿真环境

软件在环仿真在主机上运行仿真系统的全部组件，使用软件来模拟真实飞控， 并通过当地网络实现与仿真软件的连接。 整套仿真方案设置如下：

{% mermaid %} graph LR; 仿真软件-->MAVLink; MAVLink-->SITL; {% endmermaid %}

## 运行 SITL

After ensuring that the [simulation prerequisites](../setup/dev_env.md) are installed on the system, just launch: The convenience make target will compile the POSIX host build and run the simulation.

```sh
make px4_sitl_default jmavsim
```

This will bring up the PX4 shell:

```sh
[init] shell id: 140735313310464
[init] task name: px4

______  __   __    ___ 
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

Ready to fly.


pxh>
```

It will also bring up a window showing a 3D view of the [jMAVSim](https://github.com/PX4/jMAVSim) simulator:

![jMAVSim 3d View](../../assets/simulation/jmavsim.jpg)

## Taking it to the Sky

The system will start printing status information. You will be able to start flying once you have a position lock (shortly after the console displays the message: *EKF commencing GPS fusion*).

To takeoff enter the following into the console:

```sh
pxh> commander takeoff
```

You can use *QGroundControl* to fly a mission or to connect to a [joystick](#joystick).

## Usage/Configuration Options

### Set Custom Takeoff Location

The default takeoff location in can be overridden using the environment variables: `PX4_HOME_LAT`, `PX4_HOME_LON`, and `PX4_HOME_ALT`.

For example, to set the latitude, longitude and altitude:

    export PX4_HOME_LAT=28.452386
    export PX4_HOME_LON=-13.867138
    export PX4_HOME_ALT=28.5
    make px4_sitl_default jmavsim
    

### Using a Joystick {#joystick}

Joystick and thumb-joystick support are supported through *QGroundControl* ([setup instructions here](../simulation/README.md#joystickgamepad-integration)).

### Simulating a Wifi Drone

There is a special target to simulate a drone connected via Wifi on the local network:

```sh
make broadcast jmavsim
```

The simulator broadcasts its address on the local network as a real drone would do.

### Start JMAVSim and PX4 Separately

You can start JMAVSim and PX4 separately:

    ./Tools/jmavsim_run.sh
    make px4_sitl none
    

This allows a faster testing cycle (restarting jMAVSim takes significantly more time).

## Multi-Vehicle Simulation

JMAVSim can be used for multi-vehicle simulation: [Multi-Vehicle Sim with JMAVSim](../simulation/multi_vehicle_jmavsim.md).

## Extending and Customizing

To extend or customize the simulation interface, edit the files in the **Tools/jMAVSim** folder. The code can be accessed through the[jMAVSim repository](https://github.com/px4/jMAVSim) on Github.

> **Info** The build system enforces the correct submodule to be checked out for all dependencies, including the simulator. It will not overwrite changes in files in the directory, however, when these changes are committed the submodule needs to be registered in the Firmware repo with the new commit hash. To do so, `git add Tools/jMAVSim` and commit the change. This will update the GIT hash of the simulator.

## Interfacing to ROS

The simulation can be [interfaced to ROS](../simulation/ros_interface.md) the same way as onboard a real vehicle.

## Important Files

* The startup script is in the [posix-configs/SITL/init](https://github.com/PX4/Firmware/tree/master/posix-configs/SITL/init) folder and named `rcS_SIM_AIRFRAME`, the default is `rcS_jmavsim_iris`.
* The root file system (the equivalent of `/` as seen by the) is located inside the build directory: `build/px4_sitl_default/src/firmware/posix/rootfs/`