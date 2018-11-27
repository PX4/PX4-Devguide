# 다중 비행체 시뮬레이션

이 튜토리얼에서는 Gazebo로 다중 UAV를 시뮬레이션하는 방법에 대해서 설명합니다. 이것은 \(SITL\) 시뮬레이션입니다.

**Note**: Linux에서만 테스트하였습니다.

## 요구사항

* ROS indigo 이상 버전
* [MAVROS 패키지](http://wiki.ros.org/mavros)
* Gazebo 7 \([Gazebo 시뮬레이션](/simulation/gazebo.md)\) 참고
* 가장 최신 소스 [PX4/Firmware](https://github.com/PX4/Firmware)

## 빌드 및 테스트

예제 셋업을 테스트하기 위해서 아래 단계를 따라합니다.

* PX4/Firmware 코드 clone 및 SITL 코드 빌드하기
  ```
  cd Firmware_clone
  git submodule update --init --recursive
  make px4_sitl_default gazebo
  ```
* 환경 설정

  ```
  source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
  ```

* launch 파일 실행

  ```
  roslaunch px4 multi_uav_mavros_sitl.launch
  ```

## What's happening?

시뮬레이트할 비행체마다 다음과 같은 작업이 필요

* **Gazebo model**: `Firmware/Tools/sitl_gazebo/models/rotors_description/urdf/<model>_base.xacro`에 있는 `xacro` 파일로 정의되며 [여기](https://github.com/PX4/sitl_gazebo/tree/02060a86652b736ca7dd945a524a8bf84eaf5a05/models/rotors_description/urdf)를 참고하세요. 현재 모델 `xacro`파일은 base.xacro로 끝난다고 가정합니다. 이 모델은 `mavlink_udp_port`라는 인자를 가지는데 이는 gazebo가 PX4 node와 통신하는 UDP 포트를 정의하는 것입니다. 모델의 `xacro` 파일은 `sdf`모델을 생성하는데 사용하며 여기에는 여러분이 선택하는 UDP 포트를 포함하고 있습니다. UDP 포트를 정의하기 위해서는 launch 파일에 있는 `mavlink_udp_port`를 설정합니다. [여기](https://github.com/PX4/Firmware/blob/master/launch/multi_uav_mavros_sitl.launch#L48)를 참고하세요.
  **NOTE: 만약 동일한 비행체 모델을 사용하고 있다면 각 비행체에 대해서 별도의 **`xacro`** 파일을 분리할 필요는 없습니다. 동일한 **`xacro`** 파일로도 충분합니다.
  **

* **PX4 node**: 이것은 SITL PX4 app입니다. 시뮬레이터인 Gazebo와 통신하는데 Gazebo vehicle model에 정의한 동일한 UDP 포트를 사용합니다.(예로 `mavlink_udp_port`) PX4 SITL app쪽에서 UDP 포트를 설정하기 위해서는 startup 파일에서 `SITL_UDP_PRT` 파리미터를 설정합니다. [여기](https://github.com/PX4/Firmware/blob/master/posix-configs/SITL/init/ekf2/iris_1#L48)를 참고하세요. 각 비행체에 대해서 launch 파일에서 startup 파일의 경로를 지정해야 합니다. [여기](https://github.com/PX4/Firmware/blob/master/launch/multi_uav_mavros_sitl.launch#L36)를 참고하세요. startup 파일에서 각 vehicle에 대한 고유 mavlink ID를 설정하는 것을 추천합니다. [여기](https://github.com/PX4/Firmware/blob/master/posix-configs/SITL/init/ekf2/iris_2#L4)를 참고하세요.

* **MAVROS node ** \(선택사항\): 별도 MAVROS node를 launch 파일에서 실행할 수 있습니다. [여기](https://github.com/PX4/Firmware/blob/master/launch/multi_uav_mavros_sitl.launch#L85-L93)를 참고하세요. PX4 SITL app에 연결하고 ROS를 통해서 비행체를 제어가 가능합니다. startup 파일에 있는 포트들의 고유 집합에서 mavlink stream를 시작시켜야 합니다. [여기](https://github.com/PX4/Firmware/blob/master/posix-configs/SITL/init/ekf2/iris_2#L67)를 참고하세요. 포트들의 고유 집합은 MAVROS node를 launch할 때 매칭되어야 합니다. [여기](https://github.com/PX4/Firmware/blob/master/launch/multi_uav_mavros_sitl.launch#L65)를 참고하세요.

launch 파일 `multi_uav_mavros_sitl.launch`은 다음을 수행합니다.

* gazebo에 있는 world를 로딩. [여기](https://github.com/PX4/Firmware/blob/master/launch/multi_uav_mavros_sitl.launch#L21-L28)를 참고하세요.
* 각 비행체에 대해서

  * gazebo 모델을 로딩하고 PX4 SITL app 인스턴스를 실행하며 다음을 실행
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

  * mavros node를 실행
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
* 각 비행체에 대한 완전한 block은 다음과 같은 형태입니다.
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
더 많은 비행체를 시뮬레이션하기 위해서는 startup 파일에 적절히 변경한 블록들과 launch 파일의 파라미터를 추가합니다.
