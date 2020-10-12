# JSBSim Simulation

[JSBSim](http://jsbsim.sourceforge.net/index.html) is a open source flight dynamics model (FDM) that supports many operating systems, including Microsoft Windows, Apple Macintosh, Linux, IRIX, Cygwin (Unix on Windows), etc.
JSBSim is the simulation 

Features include a fully configurable aerodynamics, propulsion system that can model complex flight dynamics of an aircraft. Also rotational earth effects are modeled into the dynamics. 


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
1. (Optional) For visualization, flightgear is used. To install flightgear, refer to the [flightgear installation instructions](../simulation/flightgear.md)).

## Running the Simulation {#running}

JSBSim SITL simulation can be conveniently run through a make command as the following. This will run the PX4 SITL instance as well as the flightgear UI for visualization.
```sh
cd /path/to/Firmware
make px4_sitl jsbsim
```
If you want to run without the flightgear UI, you can add `HEADLESS=1` to the front of the make command.


The supported vehicles and `make` commands are listed below (click on the links to see the vehicle images).

Vehicle | Command
--- | ---
[Standard Plane]() | `make px4_sitl jsbsim_rascal`
[Quadrotor]() | `make px4_sitl jsbsim_quadrotor_x`
[Hexarotor]() | `make px4_sitl jsbsim_hexarotor_x`

The commands above launch a single vehicle with the full UI.
*QGroundControl* should be able to automatically connect to the simulated vehicle.


## Further Information

* [px4-jsbsim-bridge readme](https://github.com/Auterion/px4-jsbsim-bridge)
