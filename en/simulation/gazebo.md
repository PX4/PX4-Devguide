# Gazebo Simulation

[Gazebo](http://gazebosim.org) is a 3D simulation environment for autonomous robots. It supports standalone use (without ROS) or SITL + ROS.

{% youtube %}https://www.youtube.com/watch?v=qfFF9-0k4KA&vq=hd720{% endyoutube %}


{% mermaid %}
graph LR;
  Gazebo-->Plugin;
  Plugin-->MAVLink;
  MAVLink-->SITL;
{% endmermaid %}

## Installation

The installation requires to install Gazebo and our simulation plugin.

> ** Note ** Gazebo version 7 is recommended (the minimum version is Gazebo 6). If you run Linux and installed a ROS version earlier than Jade, be sure to uninstall the bundled Gazebo (sudo apt-get remove ros-indigo-gazebo) version as it is too old.

Check the [Linux](../setup/dev_env_linux.md) and [Mac](../setup/dev_env_mac.md) pages for install infos.

#### ROS Users

The PX4 SITL uses the Gazebo simulator, but does not depend on ROS. The simulation can be [interfaced to ROS](../simulation/ros_interface.md) the same way as normal flight code is.

If you plan to use PX4 with ROS, make sure to follow the [Gazebo version guide for version 7](http://gazebosim.org/tutorials?tut=ros_wrapper_versions#Gazebo7.xseries) for ROS.

#### Normal Installation

Follow the [Linux installation instructions](http://gazebosim.org/tutorials?tut=install_ubuntu&ver=7.0&cat=install) for Gazebo 7.

Make sure to have both installed: `gazebo7` and `libgazebo7-dev`.

## Running the Simulation

From within the source directory of the PX4 Firmware run the PX4 SITL with one of the airframes (Quads, planes and VTOL are supported, including optical flow):

> **Note** You can use the instructions below to keep Gazebo running and only re-launch PX4.

### Quadrotor

```sh
cd ~/src/Firmware
make posix_sitl_default gazebo
```

### Quadrotor with Optical Flow

```sh
make posix gazebo_iris_opt_flow
```

### 3DR Solo

```sh
make posix gazebo_solo
```

![](../../assets/gazebo/solo.png)

### Standard Plane

```sh
make posix gazebo_plane
```

![](../../assets/gazebo/plane.png)

### Standard VTOL

```sh
make posix_sitl_default gazebo_standard_vtol
```

![](../../assets/gazebo/standard_vtol.png)

### Tailsitter VTOL

```sh
make posix_sitl_default gazebo_tailsitter
```

![](../../assets/gazebo/tailsitter.png)

### Ackerman vehicle

```sh
make posix gazebo_rover
```

![](../../assets/gazebo/rover.png)


## Change World

The current default world is the iris.world located in the directory [worlds](https://github.com/PX4/sitl_gazebo/tree/367ab1bf55772c9e51f029f34c74d318833eac5b/worlds). The default surrounding in the iris.world uses a heightmap as ground. This ground can cause difficulty when using a distance sensor. If there are unexpected results with that heightmap, it is recommended to change the model in iris.model from uneven_ground to asphalt_plane.

## Taking it to the Sky

> ** Note ** Please refer to the [Installing Files and Code](../setup/dev_env_mac.md) guide in case you run into any errors.

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

px4 starting.


pxh>
```

> ** Note ** Right-clicking the quadrotor model allows to enable follow mode from the context menu, which is handy to keep it in view.

![](../../assets/sim/gazebo.png)

The system will print the home position once it finished intializing (`telem> home: 55.7533950, 37.6254270, -0.00`). You can bring it into the air by typing:

```sh
pxh> commander takeoff
```

> ** Note ** Joystick or thumb-joystick support is available through QGroundControl (QGC). To use manual input, put the system in a manual flight mode (e.g. POSCTL, position control). Enable the thumb joystick from the QGC preferences menu.

## Set custom takeoff location

The default takeoff location in SITL Gazebo can be overridden using environment variables.

The variables to set are: `PX4_HOME_LAT`, `PX4_HOME_LON`, and `PX4_HOME_ALT`.

As an example:
```
export PX4_HOME_LAT=28.452386
export PX4_HOME_LON=-13.867138
export PX4_HOME_ALT=28.5
make posix gazebo
```

## Starting Gazebo and PX4 separately

For extended development sessions it might be more convenient to start Gazebo and PX4 separately or even from within an IDE.

In addition to the existing cmake targets that run `sitl_run.sh` with parameters for px4 to load the correct model it creates a launcher targets named `px4_<mode>` that is a thin wrapper around original sitl px4 app. This thin wrapper simply embeds app arguments like current working directories and the path to the model file.

### How to use it

  * Run gazebo (or any other sim) server and client viewers via the terminal:
```
make posix_sitl_default gazebo_none_ide
```
  * In your IDE select `px4_<mode>` target you want to debug (e.g. `px4_iris`)
  * Start the debug session directly from IDE
  This approach significantly reduces the debug cycle time because simulator (e.g. gazebo) is always running in background and you only re-run the px4 process which is very light.

## Extending and Customizing

To extend or customize the simulation interface, edit the files in the `Tools/sitl_gazebo` folder. The code is available on the [sitl_gazebo repository](https://github.com/px4/sitl_gazebo) on Github.

> ** Note ** The build system enforces the correct GIT submodules, including the simulator. It will not overwrite changes in files in the directory.

## Interfacing to ROS

The simulation can be [interfaced to ROS](../simulation/ros_interface.md) the same way as onboard a real vehicle.
