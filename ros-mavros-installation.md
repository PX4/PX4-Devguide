
# MAVROS

The [mavros](http://wiki.ros.org/mavros#mavros.2BAC8-Plugins.sys_status) ros package enables MAVLink extendable communication between computers running ROS, MAVLink enabled autopilots, and MAVLink enabled GCS.  While MAVRos can be used to communicate with any MAVLink enabled autopilot this documentation will be in the context of enabling communication between the PX4 flight stack and a ROS enabled companion computer.

## Installation

MAVROS can be installed either from source or binary. Developers working with ROS are advised to use the source installation.

### Binary installation (Debian / Ubuntu)

Since v0.5 that programs available in precompiled debian packages for x86 and amd64 (x86\_64).
Also v0.9+ exists in ARMv7 repo for Ubuntu armhf.
Just use `apt-get` for installation:
```sh
$ sudo apt-get install ros-indigo-mavros ros-indigo-mavros-extras
```

### Source installation
**Dependencies**

This installation assumes you have a catkin workspace located at `~/catkin_ws` If you don't create one with: 
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin init
```

You will be using the ROS python tools `wstool, rosinstall,and catkin_tools` for this installation. While they may have been installed during your installation of ROS you can also install them with:
```sh
$ sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools
```

Note that while the package can be built using catkin_make the prefered method is using catkin_tools as it is a more versatile and "friendly" build tool.

If this is your first time using wstool you will need to initialize your source space with:
```sh
$ wstool init ~/catkin_ws/src
```

Now you are ready to do the build
```sh
    # 1. get source (upstream - released)
$ rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
    # alternative: latest source
$ rosinstall_generator --upstream-development mavros | tee /tmp/mavros.rosinstall

    # 2. get latest released mavlink package
    # you may run from this line to update ros-*-mavlink package
$ rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall

    # 3. Setup workspace & install deps
$ wstool merge -t src /tmp/mavros.rosinstall
$ wstool update -t src
$ rosdep install --from-paths src --ignore-src --rosdistro indigo -y

    # finally - build
$ catkin build
```
<aside class="note">
If you are installing mavros on a raspberry pi, you may get an error related to your os, when running "rosdep install ...". Add "--os=OS_NAME:OS_VERSION " to the rosdep command and replace OS_NAME with your OS name and OS_VERSION with your OS version (e.g. --os=debian:jessie).
</aside>
