# Gazebo 多机仿真

This topic explains how to simulate multiple UAV vehicles using Gazebo and SITL (Linux only). A different approach is used for simulation with and without ROS.

## Multiple Vehicle with Gazebo (No ROS) {#no_ros}

To simulate multiple iris or plane vehicles in Gazebo use the following commands in the terminal (from the root of the *Firmware* tree):

    Tools/gazebo_sitl_multiple_run.sh [-m <model>] [-n <number_of_vehicles>]
    

* `<model>`: The vehicle type/model to spawn: `iris` (default), `plane`.
* `number_of_vehicles`: The number of vehicles to spawn. Default is 3. Maximum is 255.

Each vehicle instance is allocated a unique MAVLink system id (1, 2, 3, etc.) and can be accessed from a unique remote offboard UDP port (14540, 14541, 14542, etc.).

> **Note** The 255-vehicle limitation occurs because mavlink `MAV_SYS_ID` only supports 255 vehicles in the same network The `MAV_SYS_ID` and various UDP ports are allocated in the SITL rcS: [init.d-posix/rcS](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d-posix/rcS#L108-L112)

### Video: Multiple Multicopter (Iris)

{% youtube %} https://youtu.be/Mskx_WxzeCk {% endyoutube %}

### Video: Multiple Plane

{% youtube %} https://youtu.be/aEzFKPMEfjc {% endyoutube %}

## Multiple Vehicles with ROS and Gazebo {#with_ros}

This example demonstrates a setup that opens the Gazebo client GUI showing two Iris vehicles in an empty world. You can then control the vehicles with *QGroundControl* and MAVROS in a similar way to how you would manage a single vehicle.

### Required

* Current [PX4 ROS/Gazebo development environment](../setup/dev_env_linux_ubuntu.md#rosgazebo) > **Note** At time of writing this is Ubuntu 18.04 with ROS Melodic/Gazebo 9. See also [Gazebo Simulation](../simulation/gazebo.md).
* [MAVROS package](http://wiki.ros.org/mavros)
* a clone of latest [PX4/Firmware](https://github.com/PX4/Firmware)

### Build and Test

To build an example setup, follow the step below:

1. 克隆 PX4 固件源码, 然后编译 SITL 代码 
      cd Firmware_clone
       git submodule update --init --recursive
       DONT_RUN=1 make px4_sitl_default gazebo

2. 配置系统环境变量： 
  
      source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
       export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo

3. 启动 launch 文件： ```roslaunch px4 multi_uav_mavros_sitl.launch```
  
  > **Tip** 你可以在上述 *roslaunch* 中指定 `gui:=false` ，以便不启动界面的情况下启动 gazebo。

The tutorial example opens the Gazebo client GUI showing two Iris vehicles in an empty world.

You can control the vehicles with *QGroundControl* or MAVROS in a similar way to how you would manage a single vehicle:

* *QGroundControl* will have a drop-down to select the vehicle that is "in focus"
* MAVROS requires that you include the proper namespace before the topic/service path (e.g. for `<group ns="uav1">` you'll use */uav1/mavros/mission/push*).

### What's Happening?

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

## Multiple Vehicles using SDF Models

This section shows how developers can simulate multiple vehicles using vehicle models defined in Gazebo SDF files (instead of using models defined in the ROS Xacro file, as discussed in the rest of this topic).

The steps are:

1. Install *xmlstarlet* from your Linux terminal: ```sudo apt install xmlstarlet```
2. Use *roslaunch* with the **multi_uav_mavros_sitl_sdf.launch** launch file: ```` roslaunch multi_uav_mavros_sitl_sdf.launch vehicle:=<model_file_name> ```
  
  > **Note** that the vehicle model file name argument is optional (`vehicle:=<model_file_name>`); if omitted the [plane model](https://github.com/PX4/sitl_gazebo/tree/master/models/plane) will be used by default.

This method is similar to using the xacro except that the SITL/Gazebo port number is automatically inserted by *xmstarlet* for each spawned vehicle, and does not need to be specified in the SDF file.

To add a new vehicle, you need to make sure the model can be found (in order to spawn it in Gazebo), and PX4 needs to have an appropriate corresponding startup script.

1. You can choose to do either of: 
  * modify the **single_vehicle_spawn_sdf.launch** file to point to the location of your model by changing the line below to point to your model: ```$(find px4)/Tools/sitl_gazebo/models/$(arg vehicle)/$(arg vehicle).sdf``` > **Note** Ensure you set the `vehicle` argument even if you hardcode the path to your model.
  * copy your model into the folder indicated above (following the same path convention). 
2. The `vehicle` argument is used to set the `PX4_SIM_MODEL` environment variable, which is used by the default rCS (startup script) to find the corresponding startup settings file for the model. Within PX4 these startup files can be found in the **Firmware/ROMFS/px4fmu_common/init.d-posix/** directory. For example, here is the plane model's [startup script](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d-posix/1030_plane). For this to work, the PX4 node in the launch file is passed arguments that specify the *rCS* file (**etc/init.d/rcS**) and the location of the rootfs directory (`$(find px4)/ROMFS/px4fmu_common`). For simplicity, it is suggested that the startup file for the model be placed alongside PX4's in **Firmware/ROMFS/px4fmu_common/init.d-posix/**.

## Additional Resources

* See [Simulation](../simulation/README.md) for a description of the UDP port configuration.
* See [URDF in Gazebo](http://gazebosim.org/tutorials/?tut=ros_urdf) for more information about spawning the model with xacro.
* See [RotorS](https://github.com/ethz-asl/rotors_simulator/tree/master/rotors_description/urdf) for more xacro models.