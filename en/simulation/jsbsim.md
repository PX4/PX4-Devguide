# JSBSim Simulation

[JSBSim](http://jsbsim.sourceforge.net/index.html) is a open source flight simulator ("flight dynamics model (FDM)") that runs on Microsoft Windows, Apple Macintosh, Linux, IRIX, Cygwin (Unix on Windows), etc.
Its features include: fully configurable aerodynamics and a propulsion system that can model complex flight dynamics of an aircraft.
Rotational earth effects are also modeled into the dynamics. 


**Supported Vehicles:** Plane, Quadrotor, Hexarotor

{% youtube %}https://youtu.be/y5azVNmIVyw{% endyoutube %}


> **Note** See [Simulation](/simulation/README.md) for general information about simulators, the simulation environment, and simulation configuration (e.g. supported vehicles).


## Installation (Ubuntu Linux) {#installation}

> **Note** These instructions were tested on Ubuntu 18.04

1. Install the usual [Development Environment on Ubuntu LTS / Debian Linux](../setup/dev_env_linux_ubuntu.md).
1. Install a JSBSim release from the [release page](https://github.com/JSBSim-Team/jsbsim/releases/tag/Linux): 
   ```sh
   dpkg -i JSBSim-devel_1.1.0.dev1-<release-number>.bionic.amd64.deb
   ```
1. (Optional) FlightGear may (optionally) be used for visualisation.
   To install FlightGear, refer to the [FlightGear installation instructions](../simulation/flightgear.md)).

## Running the Simulation {#running}
### Running with make{#running}

JSBSim SITL simulation can be conveniently run through a `make` command in PX4 Firmware as shown below:
```sh
cd /path/to/Firmware
make px4_sitl jsbsim
```
This is convenient for developers who are developing the firmware and evaluating the software in the jsbsim simulation.
This will run both the PX4 SITL instance and the FlightGear UI (for visualization).
If you want to run without the FlightGear UI, you can add `HEADLESS=1` to the front of the `make` command. For example:
```
HEADLESS=1 make px4_sitl jsbsim
```

The supported vehicles and `make` commands are listed below (click on the links to see the vehicle images).

Vehicle | Command
--- | ---
Standard Plane | `make px4_sitl jsbsim_rascal`
Standard Plane | `make px4_sitl jsbsim_malolo`
Quadrotor | `make px4_sitl jsbsim_quadrotor_x`
Hexarotor | `make px4_sitl jsbsim_hexarotor_x`

The commands above launch a single vehicle with the full UI.
*QGroundControl* should be able to automatically connect to the simulated vehicle.

### Running with ROS{#running}
> **Warning** This document assumes that the [ROS](../ros/README.md) environment has been setup correctly. 
  If you plan to use PX4 with ROS you **should follow the** [ROS Instructions](../simulation/ros_interface.md) to install ROS (and thereby avoid installation conflicts).

JSBSim can be useful for ROS developers that need advanced flight dynamics simulated to evaluate their software components.
```
roslaunch jsbsim_bridge px4_jsbsim_bridge.launch
```



## Further Information

* [px4-jsbsim-bridge readme](https://github.com/Auterion/px4-jsbsim-bridge)
