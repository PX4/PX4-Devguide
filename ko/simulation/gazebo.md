# 가제보 모의 시험 환경

[가제보](http://gazebosim.org) 는 개체 회피 및 컴퓨터 시각 정보 처리 시험에 일부 적합한, 강력한 자동화 로봇용 3차원 모의 시험 환경입니다. 이 페이지에서는 SITL과 단일 기체와의 활용법을 설명합니다. 가제보는 또한 [HITL](../simulation/hitl.md)과 [다중 기체 모의 시험](../simulation/multi-vehicle-simulation.md)용으로 활용할 수 있습니다.

**지원 기체:** 쿼드 ([Iris](../airframes/airframe_reference.md#copter_quadrotor_wide_3dr_iris_quadrotor) 와 [Solo](../airframes/airframe_reference.md#copter_quadrotor_x_3dr_solo), 헥스 (태풍 H480), [일반 쿼드 델타 수직 이착륙기](../airframes/airframe_reference.md#vtol_standard_vtol_generic_quad_delta_vtol), 태일시터, 항공기, 탐사선, 수중선/무인 수중선

> **Warning** 가제보 자동화 기체 제어를 목적으로 주로 [ROS](../ros/README.md), 툴킷, 외부 보드 API를 함께 활용합니다. PX4를 ROS와 활용하려면 [ROS 활용 절차](../simulation/ros_interface.md)를 **따라** ROS와 가제보를 설치해야합니다(그리고 설치 과정상 중복을 피하십시오).

{% youtube %}https://www.youtube.com/watch?v=qfFF9-0k4KA&vq=hd720{% endyoutube %}

[![Mermaid Graph: Gazebo plugin](https://mermaid.ink/img/eyJjb2RlIjoiZ3JhcGggTFI7XG4gIEdhemViby0tPlBsdWdpbjtcbiAgUGx1Z2luLS0-TUFWTGluaztcbiAgTUFWTGluay0tPlNJVEw7IiwibWVybWFpZCI6eyJ0aGVtZSI6ImRlZmF1bHQifSwidXBkYXRlRWRpdG9yIjpmYWxzZX0)](https://mermaid-js.github.io/mermaid-live-editor/#/edit/eyJjb2RlIjoiZ3JhcGggTFI7XG4gIEdhemViby0tPlBsdWdpbjtcbiAgUGx1Z2luLS0-TUFWTGluaztcbiAgTUFWTGluay0tPlNJVEw7IiwibWVybWFpZCI6eyJ0aGVtZSI6ImRlZmF1bHQifSwidXBkYXRlRWRpdG9yIjpmYWxzZX0)

<!-- original graph info
graph LR;
  Gazebo-- >Plugin;
  Plugin-- >MAVLink;
  MAVLink-- >SITL;
-->

> **Note** 모의 시험 프로그램, 모의 시험 환경, 모의 시험 설정(예: 지원 기체) 관련 일반 정보는 [모의 시험](/simulation/README.md)을 참고 하십시오.

<a id="installation"></a>

## Installation

Gazebo 9 setup is included in our standard build instructions:

* **macOS:** [맥용 개발 환경](../setup/dev_env_mac.md)
* **Linux:** [우분투 LTS / 데비안 리눅스 > 가제보, JMAVSim, NuttX(Pixhawk) 대상 개발 환경](../setup/dev_env_linux_ubuntu.md#sim_nuttx)
* **Windows:** 지원 안함.

Additional installation instructions can be found on [gazebosim.org](http://gazebosim.org/tutorials?cat=guided_b&tut=guided_b1).

## 모의 시험 환경 실행

Run a simulation by starting PX4 SITL and gazebo with the airframe configuration to load (multicopters, planes, VTOL, optical flow and multi-vehicle simulations are supported).

The easiest way to do this is to open a terminal in the root directory of the PX4 *PX4-Autopilot* repository and call `make` for the desired target. For example, to start a quadrotor simulation (the default):

```sh
cd /path/to/PX4-Autopilot
make px4_sitl gazebo
```

The supported vehicles and `make` commands are listed below (click links to see vehicle images).

> **Note** 전체 대상을 빌드하려면 `make px4_sitl list_vmd_make_targets` 명령을 실행 (하고 `gazebo_`로 시작하는 요소를 검색) 하십시오.

| 기체                                                                                                     | 명령                                     |
| ------------------------------------------------------------------------------------------------------ | -------------------------------------- |
| [쿼드로터](../simulation/gazebo_vehicles.md#quadrotor)                                                     | `make px4_sitl gazebo`                 |
| [광류 센서 장착 쿼드로터](../simulation/gazebo_vehicles.md#quadrotor_optical_flow)                               | `make px4_sitl gazebo_iris_opt_flow`   |
| [3DR 솔로 (쿼드로터)](../simulation/gazebo_vehicles.md#3dr_solo)                                             | `make px4_sitl gazebo_solo`            |
| <span id="typhoon_h480"></span>[태풍 H480 (헥스로터)](../simulation/gazebo_vehicles.md#typhoon_h480) (동영상 스트리밍 지원) | `make px4_sitl gazebo_typhoon_h480`    |
| [표준 비행체](../simulation/gazebo_vehicles.md#standard_plane)                                              | `make px4_sitl gazebo_plane`           |
| [(캐터펄트 발사형) 표준 비행체](../simulation/gazebo_vehicles.md#standard_plane_catapult)                          | `make px4_sitl gazebo_plane_catapult`  |
| [표준 수직 이착륙기](../simulation/gazebo_vehicles.md#standard_vtol)                                           | `make px4_sitl gazebo_standard_vtol`   |
| [테일시터 수직 이착륙기](../simulation/gazebo_vehicles.md#tailsitter_vtol)                                       | `make px4_sitl gazebo_tailsitter`      |
| [Ackerman 기체 (UGV/탐사선)](../simulation/gazebo_vehicles.md#ugv)                                          | `make px4_sitl gazebo_rover`           |
| [히포캠퍼스 TUHH (UUV: 무인 수중선)](../simulation/gazebo_vehicles.md#uuv)                                       | `make px4_sitl gazebo_uuv_hippocampus` |
| [보트 (USV: 무인 수면선)](../simulation/gazebo_vehicles.md#usv)                                               | `make px4_sitl gazebo_boat`            |
| [구름선 (비행선)](../simulation/gazebo_vehicles.md#airship)                                                  | `make px4_sitl gazebo_cloudship`       |

> **Note** [파일 및 코드 설치](../setup/dev_env.md) 안내서는 빌드 과정에 오류가 나타날 경우 도움이 될 참고서입니다.

The commands above launch a single vehicle with the full UI. Other options include:

* [PX4와 가제보를 개별 시작](#start_px4_sim_separately)하면 가제보 실행 상태를 유지할 수 있고 PX4만 필요할 경우 다시 실행할 수 있습니다(둘 다 다시 시작하는 것보단 빠름).
* 가제보 인터페이스를 시작하지 않는 [헤드리스 모드](#headless)로 모의시험 환경을 실행합니다(자원을 훨씬 적게 차지하며 더 빠릅니다).

## 하늘로 띄우기

The `make` commands above first build PX4, and then run it along with the Gazebo simulator.

Once PX4 has started it will launch the PX4 shell as shown below.

    ______  __   __    ___ 
    | ___ \ \ \ / /   /   |
    | |_/ /  \ V /   / /| |
    |  __/   /   \  / /_| |
    | |     / /^\ \ \___  |
    \_|     \/   \/     |_/
    
    px4 starting.
    
    INFO  [px4] Calling startup script: /bin/sh etc/init.d-posix/rcS 0
    INFO  [param] selected parameter default file eeprom/parameters_10016
    [param] Loaded: eeprom/parameters_10016
    INFO  [dataman] Unknown restart, data manager file './dataman' size is 11798680 bytes
    INFO  [simulator] Waiting for simulator to connect on TCP port 4560
    Gazebo multi-robot simulator, version 9.0.0
    Copyright (C) 2012 Open Source Robotics Foundation.
    Released under the Apache 2 License.
    http://gazebosim.org
    ...
    INFO  [ecl/EKF] 5188000: commencing GPS fusion
    

The console will print out status as PX4 loads the airframe-specific initialisation and parameter files, waits for (and connects to) the simulator. Once there is an INFO print that [ecl/EKF] is `commencing GPS fusion` the vehicle is ready to arm.

> **Note** Right-clicking the quadrotor model allows to enable follow mode from the context menu, which is handy to keep it in view.

![Gazebo UI](../../assets/simulation/gazebo/gazebo_follow.jpg)

You can bring it into the air by typing:

```sh
pxh> commander takeoff
```

## 사용법/설정 옵션

<a id="headless"></a>

### Headless Mode

Gazebo can be run in a *headless* mode in which the Gazebo UI is not launched. This starts up more quickly and uses less system resources (i.e. it is a more "lightweight" way to run the simulation).

Simply prefix the normal `make` command with `HEADLESS=1` as shown:

```bash
HEADLESS=1 make px4_sitl gazebo_plane
```

<a id="custom_takeoff_location"></a>

### Set Custom Takeoff Location

The takeoff location in SITL Gazebo can be set using environment variables. This will override both the default takeoff location, and any value [set for the world](#set_world_location).

The variables to set are: `PX4_HOME_LAT`, `PX4_HOME_LON`, and `PX4_HOME_ALT`.

For example:

    export PX4_HOME_LAT=28.452386
    export PX4_HOME_LON=-13.867138
    export PX4_HOME_ALT=28.5
    make px4_sitl gazebo
    

### 모의 시험 진행 속도 변경

The simulation speed can be increased or decreased with respect to realtime using the environment variable `PX4_SIM_SPEED_FACTOR`.

    export PX4_SIM_SPEED_FACTOR=2
    make px4_sitl_default gazebo
    

For more information see: [Simulation > Run Simulation Faster than Realtime](../simulation/README.md#simulation_speed).

### 조종기 활용

Joystick and thumb-joystick support are supported through *QGroundControl* ([setup instructions here](../simulation/README.md#joystickgamepad-integration)).

### 거리 센서 성능 개선

The current default world is [PX4/sitl_gazebo/worlds/**iris.world**](https://github.com/PX4/sitl_gazebo/tree/master/worlds)), which uses a heightmap as ground.

This can cause difficulty when using a distance sensor. If there are unexpected results we recommend you change the model in **iris.model** from `uneven_ground` to `asphalt_plane`.

<a id="gps_noise"></a>

### Simulating GPS Noise

Gazebo can simulate GPS noise that is similar to that typically found in real systems (otherwise reported GPS values will be noise-free/perfect). This is useful when working on applications that might be impacted by GPS noise - e.g. precision positioning.

GPS noise is enabled if the target vehicle's SDF file contains a value for the `gpsNoise` element (i.e. it has the line: `<gpsNoise>true</gpsNoise>`). It is enabled by default in many vehicle SDF files: **solo.sdf**, **iris.sdf**, **standard_vtol.sdf**, **delta_wing.sdf**, **plane.sdf**, **typhoon_h480**, **tailsitter.sdf**.

To enable/disable GPS noise:

1. 임의의 가제보 대상을 빌드하여 (모든 기체에 대한) SDF 파일을 만드십시오. 예를 들어: ```make px4_sitl gazebo_iris``` > **Tip** 빌드를 반복할 때 SDF 파일은 덮어쓰지 않습니다.
2. 대상 기체의 SDF 파일을 여십시오(예: **./Tools/sitl_gazebo/models/iris/iris.sdf**).
3. `gpsNoise` 항목을 찾으십시오: 
        xml
        <plugin name='gps_plugin' filename='libgazebo_gps_plugin.so'>
         <robotNamespace/>
         <gpsNoise>true</gpsNoise>
        </plugin>
    
    * 항목이 있다면, GPS 잡음 신호가 켜진 상태입니다. 다음 줄을 삭제하면 끌 수 있습니다: `<gpsNoise>true</gpsNoise>`
    * 나타나지 않는다면 GPS 잡음 신호를 꺼둔 상태입니다. (위에서 보시는 바와 같이) `gps_plugin` 섹션에 `gpsNoise` 항목을 추가하여 GPS 잡음 신호를 켤 수 있습니다.

The next time you build/restart Gazebo it will use the new GPS noise setting.

<a id="set_world"></a>

## Loading a Specific World

PX4 supports a number of [Gazebo Worlds](../simulation/gazebo_worlds.md), which are stored in [PX4/sitl_gazebo/worlds](https://github.com/PX4/sitl_gazebo/tree/master/worlds)) By default Gazebo displays a flat featureless plane, as defined in [empty.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/empty.world).

You can load any of the worlds by specifying them as the final option in the PX4 configuration target.

For example, to load the *warehouse* world, you can append it as shown:

    make px4_sitl_default gazebo_plane_cam__warehouse
    

> **Note** There are *two underscores* after the model (`plane_cam`) indicating that the default debugger is used (none). See [Building the Code > PX4 Make Build Targets](../setup/building_px4.md#make_targets).

You can also specify the full path to a world to load using the `PX4_SITL_WORLD` environment variable. This is useful if testing a new world that is not yet included with PX4.

> **Tip** If the loaded world does not align with the map, you may need to [set the world location](#set_world_location).

<a id="set_world_location"></a>

## Set World Location

The vehicle gets spawned very close to the origin of the world model at some simulated GPS location.

> **Note** The vehicle is not spawned exactly at the Gazebo origin (0,0,0), but using a slight offset, which can highlight a number of common coding issues.

If using a world that recreates a real location (e.g. a particular airport) this can result in a very obvious mismatch between what is displayed in the simulated world, and what is shown on the ground station map. To overcome this problem you can set the location of the world origin to the GPS co-ordinates where it would be in "real life".

> **Note** You can also set a [Custom Takeoff Location](#custom_takeoff_location) that does the same thing. However adding the location to the map is easier (and can still be over-ridden by setting a custom location if needed).

The location of the world is defined in the **.world** file by specifying the location of the origin using the `spherical_coordinates` tag. The latitude, longitude, elevation must all be specified (for this to be a valid).

An example can be found in the [sonoma_raceway.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/sonoma_raceway.world):

        <spherical_coordinates>
          <surface_model>EARTH_WGS84</surface_model>
          <latitude_deg>38.161479</latitude_deg>
          <longitude_deg>-122.454630</longitude_deg>
          <elevation>488.0</elevation>
        </spherical_coordinates>
    

You can test this by spawning a rover in the [Sonoma Raceway World](../simulation/gazebo_worlds.md#sonoma-raceway) using the following `make` command (note that spawning takes longer the first time as the model needs to be downloaded from the model database):

    make px4_sitl gazebo_rover__sonoma_raceway
    

The video below shows that the location of the environment is aligned with the gazebo world: {% youtube %} https://youtu.be/-a2WWLni5do {% endyoutube %}

<a id="start_px4_sim_separately"></a>

## Starting Gazebo and PX4 Separately

For extended development sessions it might be more convenient to start Gazebo and PX4 separately or even from within an IDE.

In addition to the existing cmake targets that run `sitl_run.sh` with parameters for px4 to load the correct model it creates a launcher targets named `px4_<mode>` that is a thin wrapper around original sitl px4 app. This thin wrapper simply embeds app arguments like current working directories and the path to the model file.

To start Gazebo and PX4 separately:

* Run gazebo (or any other sim) server and client viewers via the terminal specifing an `_ide` variant: 
        sh
        make px4_sitl gazebo___ide 또는 
    
        sh
        make px4_sitl gazebo_iris_ide

* In your IDE select `px4_<mode>` target you want to debug (e.g. `px4_iris`)
* Start the debug session directly from IDE

This approach significantly reduces the debug cycle time because simulator (e.g. Gazebo) is always running in background and you only re-run the px4 process which is very light.

## Simulated Survey Camera

The *Gazebo* survey camera simulates a [MAVLink camera](https://mavlink.io/en/services/camera.html) that captures geotagged JPEG images and sends camera capture information to a connected ground station. The camera also supports video streaming. It can be used to test camera capture, in particular within survey missions.

The camera emits the [CAMERA_IMAGE_CAPTURED](https://mavlink.io/en/messages/common.html#CAMERA_IMAGE_CAPTURED) message every time an image is captured. The captured images are saved to: **PX4-Autopilot/build/px4_sitle_default/tmp/frames/DSC_n_.jpg** (where *n* starts as 00000 and is iterated by one on each capture).

To simulate a plane with this camera:

    make px4_sitl_default gazebo_plane_cam
    

> **Note** 카메라에서는 다음 MAVLink 명령을 지원하고 이에 응답합니다: [MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS](https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS), [MAV_CMD_REQUEST_STORAGE_INFORMATION](https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_STORAGE_INFORMATION), [MAV_CMD_REQUEST_CAMERA_SETTINGS](https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_CAMERA_SETTINGS), [MAV_CMD_REQUEST_CAMERA_INFORMATION](https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_CAMERA_INFORMATION), [MAV_CMD_RESET_CAMERA_SETTINGS](https://mavlink.io/en/messages/common.html#MAV_CMD_RESET_CAMERA_SETTINGS), [MAV_CMD_STORAGE_FORMAT](https://mavlink.io/en/messages/common.html#MAV_CMD_STORAGE_FORMAT), [MAV_CMD_SET_CAMERA_ZOOM](https://mavlink.io/en/messages/common.html#MAV_CMD_SET_CAMERA_ZOOM), [MAV_CMD_IMAGE_START_CAPTURE](https://mavlink.io/en/messages/common.html#MAV_CMD_IMAGE_START_CAPTURE), [MAV_CMD_IMAGE_STOP_CAPTURE](https://mavlink.io/en/messages/common.html#MAV_CMD_IMAGE_STOP_CAPTURE), [MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION](https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION), [MAV_CMD_REQUEST_VIDEO_STREAM_STATUS](https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_VIDEO_STREAM_STATUS), [MAV_CMD_SET_CAMERA_MODE](https://mavlink.io/en/messages/common.html#MAV_CMD_SET_CAMERA_MODE).

<span></span>

> **Note** The simulated camera is implemented in [PX4/sitl_gazebo/src/gazebo_geotagged_images_plugin.cpp](https://github.com/PX4/sitl_gazebo/blob/master/src/gazebo_geotagged_images_plugin.cpp).

<a id="flight_termination"></a>

## Simulated Parachute/Flight Termination

*Gazebo* can be used to simulate deploying a [parachute](https://docs.px4.io/master/en/peripherals/parachute.html) during [Flight Termination](https://docs.px4.io/master/en/advanced_config/flight_termination.html) (flight termination is triggered by the PWM command that is simulated in *Gazebo*).

The `if750a` target has a parachute attached to the vehicle. To simulate the vehicle, run the following command:

    make px4_sitl gazebo_if750a
    

To put the vehicle into flight termination state, you can force it to fail a [safety check](https://docs.px4.io/master/en/config/safety.html) that has flight termination set as the failsafe action. For example, you could do this by forcing a [Geofence violation](https://docs.px4.io/master/en/config/safety.html#geofence-failsafe).

For more information see:

* [Flight Termination](https://docs.px4.io/master/en/advanced_config/flight_termination.html) 
* [Parachute](https://docs.px4.io/master/en/peripherals/parachute.html)
* [Safety Configuration (Failsafes)](https://docs.px4.io/master/en/config/safety.html)

<a id="video"></a>

## Video Streaming

PX4 SITL for Gazebo supports UDP video streaming from a Gazebo camera sensor attached to a vehicle model. When streaming is enabled, you can connect to this stream from *QGroundControl* (on UDP port 5600) and view video of the Gazebo environment from the simulated vehicle - just as you would from a real camera. The video is streamed using a *gstreamer* pipeline and can be enabled/disabled using a button in the Gazebo UI.

The Gazebo camera sensor is supported/enabled on the following frames:

* [Typhoon H480](#typhoon_h480)

### Prerequisites

*Gstreamer 1.0* is required for video streaming. The required dependencies should already have been [installed when you set up Gazebo](#installation) (they are included in the standard PX4 installation scripts/instructions for macOS and Ubuntu Linux).

> **Note** FYI only, the dependencies include: `gstreamer1.0-plugins-base`, g`streamer1.0-plugins-good`, `gstreamer1.0-plugins-bad`, `gstreamer1.0-plugins-ugly`, `libgstreamer-plugins-base1.0-dev`.

### 동영상 스트리밍 시작/중지

Video streaming is automatically started when supported by the target vehicle. For example, to start streaming video on the Typhoon H480:

    make px4_sitl gazebo_typhoon_h480
    

Streaming can be paused/restarted using the Gazebo UI *Video ON/OFF* button..

![Video ON/OFF button](../../assets/simulation/gazebo/sitl_video_stream.png)

### 가제보 동영상 보는 방법

The easiest way to view the SITL/Gazebo camera video stream is in *QGroundControl*. Simply open **Application Settings > General** and set **Video Source** to *UDP h.264 Video Stream* and **UDP Port** to *5600*:

![QGC Video Streaming Settings for Gazebo](../../assets/simulation/gazebo/qgc_gazebo_video_stream_udp.png)

The video from Gazebo should then display in *QGroundControl* just as it would from a real camera.

![QGC Video Streaming Gazebo Example](../../assets/simulation/gazebo/qgc_gazebo_video_stream_typhoon.jpg)

> **Note** The Typhoon world is not very interesting.

It is also possible to view the video using the *Gstreamer Pipeline*. Simply enter the following terminal command:

```sh
gst-launch-1.0  -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink fps-update-interval=1000 sync=false
```

### Verbose Logging

SITL fails silently when there is something wrong with the gazebo model. You can enable more verbose logging using `VERBOSE_SIM`, as shown:

    export VERBOSE_SIM=1
    make px4_sitl gazebo
    

or

    VERBOSE_SIM=1 make px4_sitl gazebo
    

## 확장 및 개별 설정

To extend or customize the simulation interface, edit the files in the `Tools/sitl_gazebo` folder. The code is available on the [sitl_gazebo repository](https://github.com/px4/sitl_gazebo) on Github.

> **Note** The build system enforces the correct GIT submodules, including the simulator. It will not overwrite changes in files in the directory.

## 추가 정보

* [ROS with Gazebo Simulation](../simulation/ros_interface.md)
* [Gazebo Octomap](../simulation/gazebo_octomap.md)