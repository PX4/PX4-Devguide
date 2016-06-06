# Interfacing the Simulation to ROS

The simulated autopilot starts a second MAVLink interface on port 14557. Connecting MAVROS to this port allows to receive all data the vehicle would expose if in real flight.

## Launching MAVROS

If an interface to ROS is wanted, the already running secondary MAVLink instance can be connected to ROS via [mavros](ros-mavros-offboard.md). To connect to a specific IP (`fcu_url` is the IP / port of SITL), use a URL in this form:

<div class="host-code"></div>

```sh
roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"
```

To connect to localhost, use this URL:

<div class="host-code"></div>

```sh
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```


## Installing Gazebo for ROS

The Gazebo ROS SITL simulation is known to work with Gazebo6 (Gazebo7 did NOT work) which can be installed via:

```sh
sudo apt-get install ros-indigo-gazebo6-ros
```

If other sensor models such as lasers are desired, the Gazebo plugins will also be needed:

```sh
sudo apt-get install ros-indigo-gazebo6-plugins
```


## Launching Gazebo with ROS wrappers

In case you would like to modify the Gazebo simulation to integrate sensors publishing directly to ROS topics e.g. the Gazebo ROS laser plugin, Gazebo must be launched with the appropriate ROS wrappers.

There are ROS launch scripts available to run the simulation wrapped in ROS:

  * [posix_sitl.launch](https://github.com/PX4/Firmware/blob/master/launch/posix_sitl.launch): plain SITL launch
  * [mavros_posix_sitl.launch](https://github.com/PX4/Firmware/blob/master/launch/mavros_posix_sitl.launch): SITL and MAVROS

To run SITL wrapped in ROS the ROS environment needs to be updated, then launch as usual:

```sh
cd <Firmware_clone>
source integrationtests/setup_gazebo_ros.bash $(pwd)
roslaunch px4 posix_sitl.launch
```

Include one of the above mentioned launch files in your own launch file to run your ROS application in the simulation.

### What's happening behind the scenes

(or how to run it manually)

```sh
no_sim=1 make posix_sitl_default gazebo
```

This should start the simulator and the console will look like this


```sh
[init] shell id: 46979166467136
[init] task name: mainapp

______  __   __    ___
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

Ready to fly.


INFO  LED::init
729 DevObj::init led
736 Added driver 0x2aba34001080 /dev/led0
INFO  LED::init
742 DevObj::init led
INFO  Not using /dev/ttyACM0 for radio control input. Assuming joystick input via MAVLink.
INFO  Waiting for initial data on UDP. Please start the flight simulator to proceed..
```

Now in a new terminal make sure you will be able to insert the Iris model through the Gazebo menus, to do this set your environment variables to include the appropriate `sitl_gazebo` folders.

```sh
cd <Firmware_clone>
source integrationtests/setup_gazebo_ros.bash $(pwd)
```

Now start Gazebo like you would when working with ROS and insert the Iris quadcopter model. Once the Iris is loaded it will automatically connect to the px4 app.

```sh
roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/sitl_gazebo/worlds/iris.world
```
