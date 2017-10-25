# OctoMap

[OctoMap 라이브러리](http://octomap.github.io/)는 3D 그리드 매핑접근법을 구현했습니다. 이 가이드에서는 [Rotors Simulator](https://github.com/ethz-asl/rotors_simulator/wiki/RotorS-Simulator)로 사용하는 방법을 다루고 있습니다.

## 설치

설치하려면 ROS, Gazebo와 Rotor 시뮬레이터 플러그인을 설치해야 합니다. [설치절차](https://github.com/ethz-asl/rotors_simulator)에 따라 Rotor Simulator에 설치합니다.

다음으로 OctoMap library를 설치합니다
<div class="host-code"></div>
```sh
	sudo apt-get install ros-indigo-octomap ros-indigo-octomap-mapping
	rosdep install octomap_mapping
	rosmake octomap_mapping
```

이제 ~/catkin_ws/src/rotors_simulator/rotors_gazebo/CMakeLists.txt를 열어서 다음 라인을 파일의 마지막에 추가합니다.
<div class="host-code"></div>
```sh
	find_package(octomap REQUIRED)
	include_directories(${OCTOMAP_INCLUDE_DIRS})
	link_libraries(${OCTOMAP_LIBRARIES})
```

~/catkin_ws/src/rotors_simulator/rotors_gazebo/package.xml을 열고 다음 라인을 추가합니다.
<div class="host-code"></div>
```sh
	<build/depend>octomap</build/depend>
	<run_depend>octomap</run_depend>
```

다음 2개 라인을 실행합니다.

> **Note** 첫번째 라인은 gedit로(기본은 vim) 기본 쉘 에디터를 변경합니다. vim에 경험이 적은 사용자에게 추천하며 그렇지 않은 경우 생략합니다.

<div class="host-code"></div>
```sh
	export EDITOR='gedit'
	rosed octomap_server octomap_tracking_server.launch
```
그리고 2개 라인을
<div class="host-code"></div>
```sh
	<param name="frame_id" type="string" value="map" />
	...
	<!--remap from="cloud_in" to="/rgbdslam/batch_clouds" /-->
```
<div class="host-code"></div>
다음처럼 변경합니다.
```sh
	<param name="frame_id" type="string" value="world" />
	...
	<remap from="cloud_in" to="/firefly/vi_sensor/camera_depth/depth/points" />
```



## 시뮬레이션 실행하기

3개 각 터미널 윈도우에서 다음 3개 라인을 실행합니다. Gazebo, Rviz, octomap 서버를 시작합니다.

<div class="host-code"></div>
```sh
	roslaunch rotors_gazebo mav_hovering_example_with_vi_sensor.launch  mav_name:=firefly
	rviz
	roslaunch octomap_server octomap_tracking_server.launch
```

Rviz에서 윈도우의 왼쪽위 'map'에서 'world'로 'Fixed Frame' 필드를 변경합니다.
이제 왼쪽 바닥에 있는 추가 버튼을 클릭해서 MarkerArray를 선택합니다. 다음으로 MarkerArray를 더블클릭하고 'Marker Topic'를 '/free_cells_vis_array'에서 '/occupied_cells_vis_array'로 변경합니다.

이제 바닥의 일부를 볼 수 있습니다.

Gazebo 창에서 붉은 로터 앞에 큐브를 넣으면 이를 Rviz에서 볼 수 있습니다.


![](../../assets/sim/octomap.png)
