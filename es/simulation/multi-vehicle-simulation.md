# Multi-Vehicle Simulation with Gazebo

This topic explains how to simulate multiple UAV vehicles using Gazebo and SITL (Linux only).

> **Tip** If you don't need a feature provided by Gazebo or ROS, [Multi-Vehicle Simulation with JMAVSim](../simulation/multi_vehicle_jmavsim.md) is easier to set up.

It demonstrates an example setup that opens the Gazebo client GUI showing two Iris vehicles in an empty world. You can then control the vehicles with *QGroundControl* and MAVROS in a similar way to how you would manage a single vehicle.

## Required

* Current [PX4 ROS/Gazebo development evironment](../setup/dev_env_linux.md#ros) > **Note** At time of writing this is Ubuntu 18.04 with ROS Melodic/Gazebo 9. See also [Gazebo Simulation](../simulation/gazebo.md).
* [MAVROS package](http://wiki.ros.org/mavros)
* a clone of latest [PX4/Firmware](https://github.com/PX4/Firmware)

## Build and Test

To build an example setup, follow the step below:

1. Clone the PX4/Firmware code, then build the SITL code 
      cd Firmware_clone
       git submodule update --init --recursive
       DONT_RUN=1 make px4_sitl_default gazebo

2. Source your environment:
  
      source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
       export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo

3. Run launch file:
  
      roslaunch px4 multi_uav_mavros_sitl.launch
  
  > **Tip** You can specify `gui:=false` in the above *roslaunch* to launch Gazebo without its UI.

The tutorial example opens the Gazebo client GUI showing two Iris vehicles in an empty world.

You can control the vehicles with *QGroundControl* or MAVROS in a similar way to how you would manage a single vehicle:

* *QGroundControl* will have a drop-down to select the vehicle that is "in focus"
* MAVROS requires that you include the proper namespace before the topic/service path (e.g. for `<group ns="uav1">` you'll use */uav1/mavros/mission/push*).

## What's Happening?

For each simulated vehicle, the following is required:

* **Gazebo model**: This is defined as `xacro` file in `Firmware/Tools/sitl_gazebo/models/rotors_description/urdf/<model>_base.xacro` see [here](https://github.com/PX4/sitl_gazebo/tree/02060a86652b736ca7dd945a524a8bf84eaf5a05/models/rotors_description/urdf). Currently, the model `xacro` file is assumed to end with base.xacro. This model should have an argument called `mavlink_udp_port` which defines the UDP port on which gazebo will communicate with PX4 node. The model's `xacro` file will be used to generate an `urdf` model that contains UDP port that you select. To define the UDP port, set the `mavlink_udp_port` in the launch file for each vehicle, see [here](https://github.com/PX4/Firmware/blob/4d0964385b84dc91189f377aafb039d10850e5d6/launch/multi_uav_mavros_sitl.launch#L37) as an example.
  
  > **Note** If you are using the same vehicle model, you don't need a separate **`xacro`** file for each vehicle. The same **`xacro`** file is adequate.

* **PX4 node**: This is the SITL PX4 app. It communicates with the simulator, Gazebo, through the same UDP port defined in the Gazebo vehicle model, i.e. `mavlink_udp_port`. To set the UDP port on the PX4 SITL app side, you need to set the `SITL_UDP_PRT` parameter in the startup file to match the `mavlink_udp_port` discussed previously, see [here](https://github.com/PX4/Firmware/blob/4d0964385b84dc91189f377aafb039d10850e5d6/posix-configs/SITL/init/ekf2/iris_2#L46). The path of the startup file in the launch file is generated based on the `vehicle` and `ID` arguments, see [here](https://github.com/PX4/Firmware/blob/4d0964385b84dc91189f377aafb039d10850e5d6/launch/multi_uav_mavros_sitl.launch#L36). The `MAV_SYS_ID` for each vehicle in the startup file, see [here](https://github.com/PX4/Firmware/blob/4d0964385b84dc91189f377aafb039d10850e5d6/posix-configs/SITL/init/ekf2/iris_2#L4), should match the `ID` for that vehicle in the launch file [here](https://github.com/PX4/Firmware/blob/4d0964385b84dc91189f377aafb039d10850e5d6/launch/multi_uav_mavros_sitl.launch#L25). This will help make sure you keep the configurations consistent between the launch file and the startup file.

* **MAVROS node** \(optional\): A seperate MAVROS node can be run in the launch file, see [here](https://github.com/PX4/Firmware/blob/4d0964385b84dc91189f377aafb039d10850e5d6/launch/multi_uav_mavros_sitl.launch#L41), in order to connect to PX4 SITL app, if you want to control your vehicle through ROS. You need to start a MAVLink stream on a unique set of ports in the startup file, see [here](https://github.com/PX4/Firmware/blob/4d0964385b84dc91189f377aafb039d10850e5d6/posix-configs/SITL/init/ekf2/iris_1#L68). Those unique set of ports need to match those in the launch file for the MAVROS node, see [here](https://github.com/PX4/Firmware/blob/4d0964385b84dc91189f377aafb039d10850e5d6/launch/multi_uav_mavros_sitl.launch#L26).

The launch file `multi_uav_mavros_sitl.launch`does the following,

* loads a world in gazebo,

        <!-- Gazebo sim -->
        <include file="$(find gazebo_ros)/launch/empty_world.launch">
            <arg name="gui" value="$(arg gui)"/>
            <arg name="world_name" value="$(arg world)"/>
            <arg name="debug" value="$(arg debug)"/>
            <arg name="verbose" value="$(arg verbose)"/>
            <arg name="paused" value="$(arg paused)"/>
        </include>
    

* for each vehicle,
  
  * creates urdf model from xacro, loads gazebo model and runs PX4 SITL app instance
          <!-- PX4 SITL and vehicle spawn -->
          <include file="$(find px4)/launch/single_vehicle_spawn.launch">
              <arg name="x" value="0"/>
              <arg name="y" value="0"/>
              <arg name="z" value="0"/>
              <arg name="R" value="0"/>
              <arg name="P" value="0"/>
              <arg name="Y" value="0"/>
              <arg name="vehicle" value="$(arg vehicle)"/>
              <arg name="rcS" value="$(find px4)/posix-configs/SITL/init/$(arg est)/$(arg vehicle)_$(arg ID)"/>
              <arg name="mavlink_tcp_port" value="4560"/>
              <arg name="ID" value="$(arg ID)"/>
          </include>
      
  
  * runs a mavros node
          <!-- MAVROS -->
          <include file="$(find mavros)/launch/px4.launch">
              <arg name="fcu_url" value="$(arg fcu_url)"/>
              <arg name="gcs_url" value=""/>
              <arg name="tgt_system" value="$(arg ID)"/>
              <arg name="tgt_component" value="1"/>
          </include>
      
  
  > **Note** The complete block for each vehicle is enclosed in a set of `<group>` tags to separate the ROS namespaces of the vehicles.

To add a third iris to this simulation there are two main components to consider:

* add `UAV3` to **multi_uav_mavros_sitl.launch** 
  * duplicate the group of either existing vehicle (`UAV1` or `UAV2`)
  * increment the `ID` arg to `3`
  * select a different port for `mavlink_udp_port` arg for communication with Gazebo
  * selects ports for MAVROS communication by modifying both port numbers in the `fcu_url` arg

* create a startup file, and change the file as follows:
  
  * make a copy of an existing iris rcS startup file (`iris_1` or `iris_2`) and rename it `iris_3`
  * `MAV_SYS_ID` value to `3`
  * `SITL_UDP_PRT` value to match that of the `mavlink_udp_port` launch file arg
  * the first `mavlink start` port and the `mavlink stream` port values to the same values, which is to be used for QGC communication
  * the second `mavlink start` ports need to match those used in the launch file `fcu_url` arg
    
    > **Note** Be aware of which port is `src` and `dst` for the different endpoints.

## Additional Resources

* See [Simulation](../simulation/README.md) for a description of the UDP port configuration.
* See [URDF in Gazebo](http://gazebosim.org/tutorials/?tut=ros_urdf) for more information about spawning the model with xacro.
* See [RotorS](https://github.com/ethz-asl/rotors_simulator/tree/master/rotors_description/urdf) for more xacro models.