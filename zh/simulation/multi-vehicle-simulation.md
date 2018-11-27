# 基于gazebo的多飞行器仿真

本主题介绍如何使用 gazebo 和 sitl (仅限 linux) 模拟多架无人机/车辆。

> **注意**如果您不需要 gazebo 或 ros 提供的功能， [ jmavsim的 Multi-车辆仿真](../simulation/multi_vehicle_jmavsim.md)更容易设置。

本文演示了一个示例设置, 该设置打开了 gazebo 客户端 界面, 在一个空旷的世界中显示了两个Iris无人机。 然后, 您可以使用 *QGroundControl地面站* 和MAVROS 控制多机, 其方式类似于您控制单机。

## 要求

* 当前的 [PX4 ros/gazebo 开发环境](../setup/dev_env_linux.md#gazebo-with-ros)。**注意** 在编写本报告时是 ubuntu 16.04 与 ros kinetic/gazebo 7。 另见 [Gazebo 模拟](/simulation/gazebo.md)。
* [MAVROS 包](http://wiki.ros.org/mavros)
* 最新 [PX4/Firmware](https://github.com/PX4/Firmware) 的克隆

## 编译和测试

若要编译示例设置, 请按照以下步骤操作:

1. 克隆 px4固件代码, 然后编译 sitl 代码 
      cd Firmware_clone
       git submodule update --init --recursive
       make px4_sitl_default
       make px4_sitl_default sitl_gazebo

2. Source your environment:
  
      source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
       export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo

3. 运行启动文件:
  
      roslaunch px4 multi_uav_mavros_sitl.launch
  
  > **注意**您可以在上述 *roslaunch* 中指定 `gui:=false`, 以便在没有 ui 的情况下启动 gazebo。

本指南设置打开了 gazebo 客户端界面, 在一个空旷的世界中显示了两个Iris无人机。

然后, 您可以使用 *QGroundControl地面站* 和MAVROS 控制多机, 其方式类似于您控制单机。

* *QGroundControl* 中有一个下拉菜单，你可以选择关注的飞行器。
* MAVROS要求你在topic/servic路径之前包含合适的命名空间，（例如，你会用到*/uav1/mavros/mission/push*）。

## 发生了什么？

对每一个仿真的飞行器，有如下要求：

* **Gazebo model**：在路径`Firmware/Tools/sitl_gazebo/models/rotors_description/urdf/<model>_base.xacro`下被定义成`xacro` 文件。看[这里](https://github.com/PX4/sitl_gazebo/tree/02060a86652b736ca7dd945a524a8bf84eaf5a05/models/rotors_description/urdf)。 目前，模型 `xacro`文件设定以 base. xacro 结尾。 此模型应该有一个名为 `mavlink_udp_port` 的参数, 该参数定义了与 px4 节点通信的 udp 端口。 模型的 `xacro` 文件将用于生成包含您选择的 udp 端口的 `urdf` 模型。 若要定义 udp 端口，请在每个飞行器的启动文件中设置 `mavlink_udp_port`，请参阅例子[here](https://github.com/PX4/Firmware/blob/4d0964385b84dc91189f377aafb039d10850e5d6/launch/multi_uav_mavros_sitl.launch#L37)。
  
  > **注意** 如果你在使用同一个飞行器模型，你不需要为每一个飞行器设置一个单独的**`xacro`**文件。 相同的 **`xacro`** 文件就可以了。

* **PX4 node**: 这是 sitl px4 应用程序。它通过在Gazebo模型中定义的同一 udp 端口 （即 `mavlink_udp_port`）与模拟器 、gazebo 进行通信。 要在 px4 sitl 应用程序端设置 udp 端口, 您需要在启动文件中设置 `SITL_UDP_PRT` 参数, 以匹配前面讨论的 `mavlink_udp_port`, 请参阅 [here](https://github.com/PX4/Firmware/blob/4d0964385b84dc91189f377aafb039d10850e5d6/posix-configs/SITL/init/ekf2/iris_2#L46)。 启动文件中的开始文件路径由参数 `vehicle`和`ID`产生，参考[这里](https://github.com/PX4/Firmware/blob/4d0964385b84dc91189f377aafb039d10850e5d6/launch/multi_uav_mavros_sitl.launch#L36)。 The `MAV_SYS_ID` for each vehicle in the startup file, see [here](https://github.com/PX4/Firmware/blob/4d0964385b84dc91189f377aafb039d10850e5d6/posix-configs/SITL/init/ekf2/iris_2#L4), should match the `ID` for that vehicle in the launch file [here](https://github.com/PX4/Firmware/blob/4d0964385b84dc91189f377aafb039d10850e5d6/launch/multi_uav_mavros_sitl.launch#L25). This will help make sure you keep the configurations consistent between the launch file and the startup file.

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
              <arg name="mavlink_udp_port" value="14560"/>
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