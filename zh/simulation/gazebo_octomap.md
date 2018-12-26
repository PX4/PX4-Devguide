# 具有 ROS/Gazebo 的 OctoMap 3D 模型

[ OctoMap 库 ](http://octomap.github.io/)是一个开源库，用于根据传感器数据生成体积 3D 环境模型。 然后，该模型数据可由无人机用于导航和避障。

本指南介绍了如何使用 *OctoMap* 与 Gazebo [Rotors Simulato ](https://github.com/ethz-asl/rotors_simulator/wiki/RotorS-Simulator) 和 ROS。

## 安装

安装需要 ROS，Gazebo 和 Rotors Simulator 插件。 按照[ Rotors Simulator instructions ](https://github.com/ethz-asl/rotors_simulator)进行安装。

接下来，安装 *OctoMap* 库：

```sh
sudo apt-get install ros-indigo-octomap ros-indigo-octomap-mapping
rosdep install octomap_mapping
rosmake octomap_mapping
```

现在，打开 ~/catkin_ws/src/rotors_simulator/rotors_gazebo/CMakeLists.txt 并在文件底部添加以下行

```sh
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})
```

打开 ~/catkin_ws/src/rotors_simulator/rotors_gazebo/package.xml 并添加以下行

```sh
<build_depend>octomap</build_depend>
<run_depend>octomap</run_depend>
```

运行以下两行：

> **Note**第一行将默认 shell 编辑器更改为 *gedit*。 对于* vim *（默认编辑器）经验不足的用户，建议使用此方法，但可以省略。

```sh
export EDITOR='gedit'
rosed octomap_server octomap_tracking_server.launch
```

并更改以下两行：

```sh
<param name="frame_id" type="string" value="map" />
...
<!--remap from="cloud_in" to="/rgbdslam/batch_clouds" /-->
```

到：

```sh
<param name="frame_id" type="string" value="world" />
...
<remap from="cloud_in" to="/firefly/vi_sensor/camera_depth/depth/points" />
```

## 运行仿真

在 *separate* 终端窗口中运行以下三行。 This opens up [Gazebo](../simulation/gazebo.md), *Rviz* and an octomap server.

```sh
roslaunch rotors_gazebo mav_hovering_example_with_vi_sensor.launch  mav_name:=firefly
rviz
roslaunch octomap_server octomap_tracking_server.launch
```

In *Rviz*, change the field 'Fixed Frame' from 'map' to 'world' in the top left of the window. Now click the add button in the bottom left and select MarkerArray. Then double click the MarkerArray and change 'Marker Topic' from '/free_cells_vis_array' to '/occupied_cells_vis_array'

Now you should see a part of the floor.

In the *Gazebo* window, insert a cube in front of the red rotors and you should see it in *Rviz*.

![OctoMap Example in Gazebo](../../assets/simulation/octomap.png)