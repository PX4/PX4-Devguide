# Simulation-In-Hardware (SIH)

The Simulation-In-Hardware (SIH) is an alternative to the [Hardware In The Loop (HIL) simulation](../simulation/hitl.md) for a quadrotor. In this setup, the controller, the state estimator, and the simulator are running on the embedded hardware. Basically, everything is running on embedded hardware. The Desktop computer is used only to display the virtual quadrotor.

![Simulator MAVLink API](../../assets/diagrams/SIH_diagram.png)

The SIH provides two benefits over the HITL:

- It ensures synchronous timing by avoiding the bidirectional connection to the computer. It means the user does not have to worry about having a powerful or real-time desktop computer.

- The whole simulation remains inside the PX4 environment. It means developers familiar with PX4 willing to incorporate their own mathematical model into the simulator can do it easily. They can, for instance, modify the aerodynamic model, or noise level of the sensors, or even add a sensor to be simulated.

The SIH can be used by new PX4 users to get familiar with PX4 and the different modes and features, and of course to learn to fly a quadrotor with the real Radio.

The dynamic model is described in this [pdf report](../../assets/simulation/SIH_dynamic_model.pdf).

Furthermore, the physical parameters representing the vehicle (such as mass, inertia, and maximum thrust force) can easily be modified from the [SIH parameters](../advanced/parameter_reference.md#simulation-in-hardware).

## Requirements

To run the SIH, you will need a [PX4 hardware autopilot](https://docs.px4.io/en/flight_controller/). If you are planning to use a [radio control transmitter and receiver pair](https://docs.px4.io/en/getting_started/rc_transmitter_receiver.html) you should have that too. Alternatively, using QGroundControl, a [joystick](https://docs.qgroundcontrol.com/en/SetupView/Joystick.html) can be used to emulate a radio control system.

The SIH is compatible with all FMU boards except FMUv2. The SIH is available on the firmware master branch and release version v1.9.0 and above.

## Setting up SIH

Running the SIH is as easy as selecting an airframe. Plug the autopilot to the desktop computer with a USB cable, let it boot, then using a ground control station select the [SIH airframe](../airframes/airframe_reference.md#simulation-copter). The autopilot will then reboot.

When the SIH airframe is selected, the SIH module starts by itself, the vehicle should be displayed on the ground control station map.

## Setting up the display

The simulated quadrotor can be displayed in jMAVSim (coming soon).

## Credits

The SIH was developed by Coriolis g Corporation, a Canadian company developing a new type of Vertical Takeoff and Landing (VTOL) Unmanned Aerial Vehicles (UAV) based on passive coupling systems.

Specialized in dynamics, control, and real-time simulation, they provide the SIH as a simple simulator for quadrotors released for free under BSD license.

Discover their current platform at [www.vogi-vtol.com](http://www.vogi-vtol.com/).