---
translated_page: https://github.com/PX4/Devguide/blob/master/en/simulation/sitl.md
translated_sha: b522243efef9deb5e2d3ae7bd03ae9ed0eee3418
---

# Multi-Vehicle Simulation

This tutorial explains how to simulate multi UAV in Gazebo. This is a software in the loop \(SITL\) simulation.

**Note**: This is tested only in Linux.

## Required

* ROS indigo or higher 
* [MAVROS package](http://wiki.ros.org/mavros)
* Gazebo 7 \(see [Gazebo Simulation](/simulation/gazebo.md)\)
* a clone of latest [PX4/Firmware](https://github.com/PX4/Firmware)

## Build and test

To test an example setup, follow the below steps,

* clone the PX4/Firmware code, then build the SITL code
  ```
  cd Firmware_clone
  git submodule update --init --recursive
  make posix_sitl_default gazebo
  ```
* source your environment,

  ```
  source Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
  ```

* run launch file,

  ```
  roslaunch px4 multi_uav_mavros_sitl.launch
  ```

## What's happening?

For each simulated vehicle, the following is required,

* **Gazebo model**: This is defined as `xacro` file in `Firmware/Tools/sitl_gazebo/models/rotors_description/urdf/<model>_base.xacro` see [here](https://github.com/PX4/sitl_gazebo/tree/02060a86652b736ca7dd945a524a8bf84eaf5a05/models/rotors_description/urdf). Currently, the model `xacro` file is assumed to end with base.xacro. This model should have an argument called  `mavlink_udp_port` which defines the UDP port on which gazebo will communicate with PX4 node. The model's `xacro` file will be used to generate an `sdf` model that contains UDP port that you select. To define the UDP port, set the `mavlink_udp_port` in the launch file, see [here](https://github.com/PX4/Firmware/blob/master/launch/multi_uav_mavros_sitl.launch#L48).  
  **NOTE: If you are using the same vehicle model, you don't need  a separate **`xacro`** file for each vehicle. The same **`xacro`** file is adequate.                
  **

* **PX4 node**: this is the SITL PX4 app. It communicates with the simulator, Gazebo, through the same UDP port defined in the Gazebo vehicle model, i.e. `mavlink_udp_port`. To set the UDP port on the PX4 SITL app side, you need to set the `SITL_UDP_PRT` parameter in the startup file, see [here](https://github.com/PX4/Firmware/blob/master/posix-configs/SITL/init/ekf2/iris_1#L48). You also need to specify the path of the startup file, for each vehicle, in the launch file, see [here](https://github.com/PX4/Firmware/blob/master/launch/multi_uav_mavros_sitl.launch#L36). It is recommended that you set a unique mavlink ID for each vehicle in the startup file, see [here](https://github.com/PX4/Firmware/blob/master/posix-configs/SITL/init/ekf2/iris_2#L4).

* **MAVROS node ** \(optional\): A seperate MAVROS node can be run in the launch file, see [here](https://github.com/PX4/Firmware/blob/master/launch/multi_uav_mavros_sitl.launch#L85-L93), in order to connect to PX4 SITL app, if you want to control your vehcile through ROS. You need to start a mavlink stream on a unique set of ports in the startup file, see [here](https://github.com/PX4/Firmware/blob/master/posix-configs/SITL/init/ekf2/iris_2#L67).  Those unique set of ports should matched when you launch MAVROS node, see [here](https://github.com/PX4/Firmware/blob/master/launch/multi_uav_mavros_sitl.launch#L65).

The launch file `multi_uav_mavros_sitl.launch`does the following,

* loads a world in gazebo, see [here](https://github.com/PX4/Firmware/blob/master/launch/multi_uav_mavros_sitl.launch#L21-L28).
* for each vehicle,

  * loads gazebo model and runs PX4 SITL app instance, runs
  ```

                <arg name="fcu_url" default="udp://:14540@localhost:14557"/>
                <arg name="gcs_url" value=""/>
                <arg name="tgt_system" value="1"/> 
                <arg name="tgt_component" value="1"/>
                <arg name="rcS1" default="$(find px4)/posix-configs/SITL/init/$(arg est)/$(arg vehicle)_1"/>
                <arg name="ID" value="1"/>

                <include file="$(find px4)/launch/single_vehcile_spawn.launch">
                    <arg name="x" value="0"/>
                    <arg name="y" value="0"/>
                    <arg name="z" value="0"/>
                    <arg name="R" value="0"/>
                    <arg name="P" value="0"/>
                    <arg name="Y" value="0"/>
                    <arg name="vehicle" value="$(arg vehicle)"/>
                    <arg name="rcS" value="$(arg rcS1)"/>
                    <arg name="mavlink_udp_port" value="14560"/>
                    <arg name="ID" value="$(arg ID)"/>
                </include>
  ```

  * runs a mavros node,
    ```
            <include file="$(find mavros)/launch/node.launch">
            <arg name="pluginlists_yaml" value="$(arg pluginlists_yaml)" />
            <arg name="config_yaml" value="$(arg config_yaml)" />

            <arg name="fcu_url" value="$(arg fcu_url)" />
            <arg name="gcs_url" value="$(arg gcs_url)" />
            <arg name="tgt_system" value="$(arg tgt_system)" />
            <arg name="tgt_component" value="$(arg tgt_component)" />
        </include>
    ```
* The complete block for each vehicle looks like the following,
    ```
        <!-- UAV2 iris_2 -->
    <group ns="uav2">
        <arg name="fcu_url" default="udp://:14541@localhost:14559"/>
        <arg name="gcs_url" value=""/>
        <arg name="tgt_system" value="2"/> 
        <arg name="tgt_component" value="1"/>
        <arg name="rcS2" default="$(find px4)/posix-configs/SITL/init/$(arg est)/$(arg vehicle)_2"/>
        <arg name="ID" value="2"/>

        <include file="$(find px4)/launch/single_vehcile_spawn.launch">
            <arg name="x" value="1"/>
            <arg name="y" value="0"/>
            <arg name="z" value="0"/>
            <arg name="R" value="0"/>
            <arg name="P" value="0"/>
            <arg name="Y" value="0"/>
            <arg name="vehicle" value="$(arg vehicle)"/>
            <arg name="rcS" value="$(arg rcS2)"/>
            <arg name="mavlink_udp_port" value="14562"/>
            <arg name="ID" value="$(arg ID)"/>
        </include>

        <include file="$(find mavros)/launch/node.launch">
            <arg name="pluginlists_yaml" value="$(arg pluginlists_yaml)" />
            <arg name="config_yaml" value="$(arg config_yaml)" />

            <arg name="fcu_url" value="$(arg fcu_url)" />
            <arg name="gcs_url" value="$(arg gcs_url)" />
            <arg name="tgt_system" value="$(arg tgt_system)" />
            <arg name="tgt_component" value="$(arg tgt_component)" />
        </include>
    </group>
    ```
    To simulate more vehicles, add more blocks with the appropriate changes in the startup files, and launch file's parameters.