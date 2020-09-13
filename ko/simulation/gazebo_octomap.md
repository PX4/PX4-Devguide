# ROS/가제보용 옥토맵 3D 모델

[옥토맵(OctoMap) 라이브러리](http://octomap.github.io/)는 센서 데이터로부터 체적 기반 3D 환경 모델을 만드는 오픈소스 라이브러리입니다. 이 모델 데이터는 드론의 지형 탐색과 충돌 방지 용도로 활용할 수 있습니다.

이 안내서에서는 가제보 [탐사선 모의 시험 환경](https://github.com/ethz-asl/rotors_simulator/wiki/RotorS-Simulator)과 ROS를 옥토맵과 활용하는 방법을 다룹니다.

## 설치

설치 과정에서 ROS, 가제보, 탐사선 모의 시험 환경 플러그인이 필요합니다. 설치하려면 [탐사선 모의 시험환경 구축 절차](https://github.com/ethz-asl/rotors_simulator)를 따르십시오.

그 다음 *옥토맵* 라이브러리를 설치하십시오:

```sh
sudo apt-get install ros-indigo-octomap ros-indigo-octomap-mapping
rosdep install octomap_mapping
rosmake octomap_mapping
```

이제, ~/catkin_ws/src/rotors_simulator/rotors_gazebo/CMakeLists.txt 파일을 려고, 다음 줄을 파일 하단에 추가하십시오.

```sh
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})
```

~/catkin_ws/src/rotors_simulator/rotors_gazebo/package.xml 파일을 열고 다음 줄을 추가하십시오

```sh
<build_depend>octomap</build_depend>
<run_depend>octomap</run_depend>
```

다음 두 줄을 실행하십시오:

> **Note** 첫번째 줄은 기본 셸 편집기를 *gedit*로 바꿉니다. *vim* (기본 편집기)에 익숙하지 않은 사용자에게 추천합니다만, 생략할 수 있습니다.

```sh
export EDITOR='gedit'
rosed octomap_server octomap_tracking_server.launch
```

그리고 다음 두 줄의 내용을 바꾸십시오:

```sh
<param name="frame_id" type="string" value="map" />
...
<!--remap from="cloud_in" to="/rgbdslam/batch_clouds" /-->
```

을:

```sh
<param name="frame_id" type="string" value="world" />
...
<remap from="cloud_in" to="/firefly/vi_sensor/camera_depth/depth/points" />
```

## 모의 시험 환경 실행

다음 세 줄을 *개별* 터미널 창에서 실행하십시오. 이 명령은 [가제보](../simulation/gazebo.md), *Rviz*, 옥토맵 서버 프로그램을 엽니다.

```sh
roslaunch rotors_gazebo mav_hovering_example_with_vi_sensor.launch  mav_name:=firefly
rviz
roslaunch octomap_server octomap_tracking_server.launch
```

In *Rviz*, change the field 'Fixed Frame' from 'map' to 'world' in the top left of the window. Now click the add button in the bottom left and select MarkerArray. Then double click the MarkerArray and change 'Marker Topic' from '/free_cells_vis_array' to '/occupied_cells_vis_array'

Now you should see a part of the floor.

In the *Gazebo* window, insert a cube in front of the red rotors and you should see it in *Rviz*.

![OctoMap Example in Gazebo](../../assets/simulation/octomap.png)