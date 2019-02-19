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

* **PX4 node**: 这是 sitl px4 应用程序。它通过在Gazebo模型中定义的同一 udp 端口 （即 `mavlink_udp_port`）与模拟器 、gazebo 进行通信。 要在 px4 sitl 应用程序端设置 udp 端口, 您需要在启动文件中设置 `SITL_UDP_PRT` 参数, 以匹配前面讨论的 `mavlink_udp_port`, 请参阅 [here](https://github.com/PX4/Firmware/blob/4d0964385b84dc91189f377aafb039d10850e5d6/posix-configs/SITL/init/ekf2/iris_2#L46)。 启动文件中的开始文件路径由参数 `vehicle`和`ID`产生，参考[这里](https://github.com/PX4/Firmware/blob/4d0964385b84dc91189f377aafb039d10850e5d6/launch/multi_uav_mavros_sitl.launch#L36)。 开始文件中的每一个飞行器的`MAV_SYS_ID`参数都要与启动文件中的`ID`相匹配。参考[这里](https://github.com/PX4/Firmware/blob/4d0964385b84dc91189f377aafb039d10850e5d6/launch/multi_uav_mavros_sitl.launch#L25)。 这样能够帮助你确保启动文件和开始文件中的设置相同。

* **MAVROS node**（可选）: 如果要通过 ros 控制车辆, 可以在启动文件中运行一个单独的 mavros 节点， 请参阅 [here](https://github.com/PX4/Firmware/blob/4d0964385b84dc91189f377aafb039d10850e5d6/launch/multi_uav_mavros_sitl.launch#L41), 以便连接到 px4 sitl 应用程序。 您需要在启动文件中一些特殊的端口上启动 mavlink 流, 请参阅 [这里](https://github.com/PX4/Firmware/blob/4d0964385b84dc91189f377aafb039d10850e5d6/posix-configs/SITL/init/ekf2/iris_1#L68)。 这些特殊端口需要与launch文件中为MAVROS节点设置的相符合。参考[这里](https://github.com/PX4/Firmware/blob/4d0964385b84dc91189f377aafb039d10850e5d6/launch/multi_uav_mavros_sitl.launch#L26)。

启动文件 `multi_uav_mavros_sitl.launch`做了以下内容,

* 在gazebo中加载一个世界

        <!-- Gazebo sim -->
        <include file="$(find gazebo_ros)/launch/empty_world.launch">
            <arg name="gui" value="$(arg gui)"/>
            <arg name="world_name" value="$(arg world)"/>
            <arg name="debug" value="$(arg debug)"/>
            <arg name="verbose" value="$(arg verbose)"/>
            <arg name="paused" value="$(arg paused)"/>
        </include>
    

* 对于每个飞行器来说
  
  * 从 xacro 创建 urdf 模型, 加载gazebo模型并运行 px4 sitl 应用程序实例
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
      
  
  * 运行mavros节点
          <!-- MAVROS -->
          <include file="$(find mavros)/launch/px4.launch">
              <arg name="fcu_url" value="$(arg fcu_url)"/>
              <arg name="gcs_url" value=""/>
              <arg name="tgt_system" value="$(arg ID)"/>
              <arg name="tgt_component" value="1"/>
          </include>
      
  
  > **Note**每个飞行器的完整块都包含在一组 `<group>` 标签中, 以区分飞行器的 ros 空间命名。

要在此模拟中添加第三个iris四旋翼, 需要考虑两个主要部分：

* 把`UAV3` 添加到**multi_uav_mavros_sitl.launch** 
  * 复制已经存在的四旋翼(`UAV1` 或者 `UAV2`)
  * 把 `ID` 改为 `3`
  * 与gazebo的通信，选择一个不同的 `mavlink_udp_port`端口
  * MAVROS通信端口选择是通过在`fcu_url` 中修改两个端口号。

* 创建一个开始文件，并按照如下方式修改：
  
  * 复制已存在的iris rcs启动文件，(`iris_1` 或 `iris_2`) ，重命名为`iris_3`
  * `MAV_SYS_ID` 值改为`3`
  * `SITL_UDP_PRT` 的值与 `mavlink_udp_port`相一致。
  * 第一个`mavlink start` 端口和`mavlink stream`端口值设置为相同值，用于和QGC通信。
  * 第二个`mavlink start` 端口值应与启动文件 `fcu_url` 中的值一致。
    
    > **Note**注意端口 `src` 和`dst`是不同的

## 其他资源

* 更多UDP端口配置请参考 [Simulation](../simulation/README.md)。
* 更多关于xacro多模型的信息请参考 [URDF in Gazebo](http://gazebosim.org/tutorials/?tut=ros_urdf)
* 更过xacro模型请参考[RotorS](https://github.com/ethz-asl/rotors_simulator/tree/master/rotors_description/urdf)。