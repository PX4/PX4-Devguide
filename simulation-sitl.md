# Software in the Loop (SITL) Simulation

Software in the Loop Simulation runs the complete system on the host machine and simulates the autopilot. It connects via local network to the simulator. The setup looks like this:

{% mermaid %}
graph LR;
  Simulator-->MAVLink;
  MAVLink-->SITL;
{% endmermaid %}

## Running SITL

After ensuring that the [simulation prerequisites](starting-installing.md) are installed on the system, just launch: The convenience make target will compile the POSIX host build and run the simulation.

```sh
make posix_sitl_default jmavsim
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

## Important Files

  * The startup script is in the [posix-configs/SITL/init](https://github.com/PX4/Firmware/tree/master/posix-configs/SITL/init) folder and named `rcS_SIM_AIRFRAME`, the default is `rcS_jmavsim_iris`.
  * The root file system (the equivalent of `/` as seen by the) is located inside the build directory: `build_posix_sitl_default/src/firmware/posix/rootfs/`

## Taking it to the Sky

And a window with the 3D view of the [jMAVSim](http://github.com/PX4/jMAVSim.git) simulator:

![](images/sim/jmavsim.png)

The system will print the home position once it finished intializing (`telem> home: 55.7533950, 37.6254270, -0.00`). You can bring it into the air by typing:

```sh
pxh> commander takeoff
```

> **Info** Joystick or thumb-joystick support is available through QGroundControl (QGC). To use manual input, put the system in a manual flight mode (e.g. POSCTL, position control). Enable the thumb joystick from the QGC preferences menu.

## Simulating a Wifi Drone

There is a special target to simulate a drone connected via Wifi on the local network:

```sh
make broadcast jmavsim
```

The simulator broadcasts his address on the local network as a real drone would do.

## Extending and Customizing

To extend or customize the simulation interface, edit the files in the `Tools/jMAVSim` folder. The code can be accessed through the[jMAVSim repository](https://github.com/px4/jMAVSim) on Github.

> ** Info ** The build system enforces the correct submodule to be checked out for all dependencies, including the simulator. It will not overwrite changes in files in the directory, however, when these changes are committed the submodule needs to be registered in the Firmware repo with the new commit hash. To do so, `git add Tools/jMAVSim` and commit the change. This will update the GIT hash of the simulator.

## Interfacing to ROS

The simulation can be [interfaced to ROS](simulation-ros-interface.md) the same way as onboard a real vehicle.
