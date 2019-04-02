# Simulation

Simulators allow PX4 flight code to control a computer modeled vehicle in a simulated "world". You can interact with this vehicle just as you might with a real vehicle, using *QGroundControl*, an offboard API, or a radio controller/gamepad.

> **Tip** Simulation is a quick, easy, and most importantly, *safe* way to test changes to PX4 code before attempting to fly in the real world. It is also a good way to start flying with PX4 when you haven't yet got a vehicle to experiment with.

PX4 supports both *Software In the Loop (SITL)* simulation, where the flight stack runs on computer (either the same computer or another computer on the same network) and *Hardware In the Loop (HITL)* simulation using a simulation firmware on a real flight controller board.

Information about available simulators and how to set them up are provided in the next section. The other sections provide general information about how the simulator works, and are not required to *use* the simulators. 


## Supported Simulators

The following simulators work with PX4 for HITL and/or SITL simulation.

Simulator |Description
---|---
[Gazebo](../simulation/gazebo.md) | <p><strong>This simulator is highly recommended.</strong></p><p>A powerful 3D simulation environment that is particularly suitable for testing object-avoidance and computer vision. It can also be used for <a href="../simulation/multi-vehicle-simulation.md">multi-vehicle simulation</a> and is commonly used with <a href="../simulation/ros_interface.md">ROS</a>, a collection of tools for automating vehicle control. </p><p><strong>Supported Vehicles:</strong> Quad (<a href="../airframes/airframe_reference.md#copter_quadrotor_wide_3dr_iris_quadrotor">Iris</a> and <a href="../airframes/airframe_reference.md#copter_quadrotor_x_3dr_solo">Solo</a>), Hex (Typhoon H480), <a href="../airframes/airframe_reference.md#vtol_standard_vtol_generic_quad_delta_vtol">Generic quad delta VTOL</a>, Tailsitter, Plane, Rover, Submarine (coming soon!) </p> 
[jMAVSim](../simulation/jmavsim.md) | A simple multirotor simulator that allows you to fly *copter* type vehicles around a simulated world. <p>It is easy to set up and can be used to test that your vehicle can take off, fly, land, and responds appropriately to various fail conditions (e.g. GPS failure). It can also be used for <a href="../simulation/multi_vehicle_jmavsim.md">multi-vehicle simulation</a>.</p><p><strong>Supported Vehicles:</strong> Quad</p>
[AirSim](../simulation/airsim.md) | A cross platform simulator that provides physically and visually realistic simulations. This simulator is resource intensive, and requires a very significantly more powerful computer than the other simulators described here. <p><strong>Supported Vehicles:</strong> Iris (MultiRotor model and a configuration for PX4 QuadRotor in the X configuration).</p>
[XPlane](../simulation/hitl.md) (HITL only)| A comprehensive and powerful fixed-wing flight simulator that offers very realistic flight models.<br><p><strong>Supported Vehicles:</strong> Plane</p>

Instructions for how to setup and use the simulators are in the topics linked above.


---
The remainder of this topic is a "somewhat generic" description of how the simulation infrastructure works. It is not required to *use* the simulators.


## Simulator MAVLink API

All simulators communicate with PX4 using the Simulator MAVLink API. This API defines a set of MAVLink messages that supply sensor data from the simulated world to PX4 and return motor and actuator values from the flight code that will be applied to the simulated vehicle. The image below shows the message flow.

![Simulator MAVLink API](../../assets/simulation/px4_simulator_messages.png)

> **Note** A SITL build of PX4 uses [simulator_mavlink.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/simulator/simulator_mavlink.cpp) to handle these messages while a hardware build in HIL mode uses [mavlink_receiver.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.cpp). Sensor data from the simulator is written to PX4 uORB topics. All motors / actuators are blocked, but internal software is fully operational.

The messages are described below (see links for specific detail).

Message | Direction | Description
--- | --- | ---
[MAV_MODE:MAV_MODE_FLAG_HIL_ENABLED](https://mavlink.io/en/messages/common.html#MAV_MODE_FLAG_HIL_ENABLED) | NA | Mode flag when using simulation. All motors/actuators are blocked, but internal software is fully operational.
[HIL_ACTUATOR_CONTROLS](https://mavlink.io/en/messages/common.html#HIL_ACTUATOR_CONTROLS) | PX4 to Sim | PX4 control outputs (to motors, actuators).
[HIL_SENSOR](https://mavlink.io/en/messages/common.html#HIL_SENSOR) | Sim to PX4 | Simulated IMU readings in SI units in NED body frame.
[HIL_GPS](https://mavlink.io/en/messages/common.html#HIL_GPS) | Sim to PX4 | The simulated GPS RAW sensor value.
[HIL_OPTICAL_FLOW](https://mavlink.io/en/messages/common.html#HIL_OPTICAL_FLOW) | Sim to PX4 | Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)
[HIL_STATE_QUATERNION](https://mavlink.io/en/messages/common.html#HIL_STATE_QUATERNION) | Sim to PX4 | Contains the actual "simulated" vehicle position, attitude, speed etc. This can be logged and compared to PX4's estimates for analysis and debugging (for example, checking how well an estimator works for noisy (simulated) sensor inputs).
[HIL_RC_INPUTS_RAW](https://mavlink.io/en/messages/common.html#HIL_RC_INPUTS_RAW) | Sim to PX4 | The RAW values of the RC channels received.


## Default PX4 MAVLink UDP Ports

By default, PX4 uses commonly established UDP ports for MAVLink communication with ground control stations (e.g. *QGroundControl*), Offboard APIs (e.g. Dronecode SDK, MAVROS) and simulator APIs (e.g. Gazebo). These ports are:

* Port **14540** is used for communication with offboard APIs. Offboard APIs are expected to listen for connections on this port.
* Port **14550** is used for communication with ground control stations. GCS are expected to listen for connections on this port. *QGroundControl* listens to this port by default.
* Port **14560** is used for communication with simulators. PX4 listens to this port, and simulators are expected to initiate the communication by broadcasting data to this port.

> **Note** The ports for the GCS and offboard APIs are set in configuration files, while the simulator broadcast port is hard-coded in the simulation MAVLink module. 


## SITL Simulation Environment

The diagram below shows a typical SITL simulation environment for any of the supported simulators. The different parts of the system connect via UDP, and can be run on either the same computer or another computer on the same network.

* PX4 uses a simulation-specific module to listen on UDP port 14560. Simulators connect to this port, then exchange information using the [Simulator MAVLink API](#simulator-mavlink-api) described above. PX4 on SITL and the simulator can run on either the same computer or different computers on the same network.
* PX4 uses the normal MAVLink module to connect to GroundStations (which listen on port 14550) and external developer APIs like Dronecode SDK or ROS (which listen on port 14540).
* A serial connection is used to connect Joystick/Gamepad hardware via *QGroundControl*.

![PX4 SITL overview](../../assets/simulation/px4_sitl_overview.png)

If you use the normal build system SITL `make` configuration targets (see next section) then both SITL and the Simulator will be launched on the same computer and the ports above will automatically be configured. You can configure additional MAVLink UDP connections and otherwise modify the simulation environment in the build configuration and initialisation files.


### Starting/Building SITL Simulation

The build system makes it very easy to build and start PX4 on SITL, launch a simulator, and connect them.
The syntax (simplified) looks like this:
```
make px4_sitl simulator[_vehicle-model]
```
where `simulator` is `gazebo`, `jmavsim` or some other simulator, and vehicle-model is a particular vehicle type supported by that simulator ([jMAVSim](../simulation/jmavsim.md) only supports multicopters, while [Gazebo](../simulation/gazebo.md) supports many different types).

A number of examples are shown below, and there are many more in the individual pages for each of the simulators:

```sh
# Start Gazebo with plane
make px4_sitl gazebo_plane

# Start Gazebo with iris and optical flow
make px4_sitl gazebo_iris_opt_flow

# Start JMavSim with iris (default vehicle model)
make px4_sitl jmavsim
```

The simulation can be further configured via environment variables:
- `PX4_ESTIMATOR`: This variable configures which estimator to use.
  Possible options are: `ekf2` (default), `lpe`, `inav`. 
  It can be set via `export PX4_ESTIMATOR=lpe` before running the simulation.

The syntax described here is simplified, and there are many other options that you can configure via *make* - for example, to set that you wish to connect to an IDE or debugger. For more information see: 
[Building the Code > PX4 Make Build Targets](../setup/building_px4.md#make_targets).


### Startup Scripts {#scripts}

Scripts are used to control which parameter settings to use or which modules to start.
They are located in the [ROMFS/px4fmu_common/init.d-posix](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d-posix) directory, the `rcS` file is the main entry point. See [System Startup](../concept/system_startup.md) for more information.


## HITL Simulation Environment

With Hardware-in-the-Loop (HITL) simulation the normal PX4 firmware is run on real hardware. The HITL Simulation Environment in documented in: [HITL Simulation](../simulation/hitl.md).


## Joystick/Gamepad Integration

*QGroundControl* desktop versions can connect to a USB Joystick/Gamepad and send its movement commands and button presses to PX4 over MAVLink. This works on both SITL and HITL simulations, and allows you to directly control the simulated vehicle. If you don't have a joystick you can alternatively control the vehicle using QGroundControl's onscreen virtual thumbsticks.

For setup information see the *QGroundControl User Guide*:
* [Joystick Setup](https://docs.qgroundcontrol.com/en/SetupView/Joystick.html)
* [Virtual Joystick](https://docs.qgroundcontrol.com/en/SettingsView/VirtualJoystick.html)

<!-- FYI Airsim info on this setting up remote controls: https://github.com/Microsoft/AirSim/blob/master/docs/remote_controls.md -->


## Camera Simulation

PX4 supports capture of both still images and video from within the [Gazebo](../simulation/gazebo.md) simulated environment. This can be enabled/set up as described in [Gazebo > Video Streaming](../simulation/gazebo.md#video-streaming).

The simulated camera is a gazebo plugin that implements the [MAVLink Camera Protocol](https://mavlink.io/en/protocol/camera.html)<!-- **Firmware/Tools/sitl_gazebo/src/gazebo_geotagged_images_plugin.cpp -->. PX4 connects/integrates with this camera in *exactly the same way* as it would with any other MAVLink camera:
1. [TRIG_INTERFACE](../advanced/parameter_reference.md#TRIG_INTERFACE) must be set to `3` to configure the camera trigger driver for use with a MAVLink camera
   > **Tip** In this mode the driver just sends a [CAMERA_TRIGGER](https://mavlink.io/en/messages/common.html#CAMERA_TRIGGER) message whenever an image capture is requested. For more information see [Camera](https://docs.px4.io/en/peripherals/camera.html).
1. PX4 must forward all camera commands between the GCS and the (simulator) MAVLink Camera. You can do this by starting [mavlink](../middleware/modules_communication.md#mavlink) with the `-f` flag as shown, specifying the UDP ports for the new connection.
   ```
   mavlink start -u 14558 -o 14530 -r 4000 -f -m camera 
   ```
   > **Note** More than just the camera MAVLink messages will be forwarded, but the camera will ignore those that it doesn't consider relevant.

The same approach can be used by other simulators to implement camera support.

