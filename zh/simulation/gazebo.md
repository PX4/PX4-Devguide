# Gazebo 仿真

[ Gazebo ](http://gazebosim.org)是用于自主机器人的强大3D模拟环境，其特别适用于测试物体避障和计算机视觉。 本文描述了如何使用它来进行单机的软件在环仿真。 Gazebo 也可以适用于 [硬件在环仿真](../simulation/hitl.md) 和 [多机仿真](../simulation/multi-vehicle-simulation.md) 。

**Supported Vehicles:** Quad ([Iris](../airframes/airframe_reference.md#copter_quadrotor_wide_3dr_iris_quadrotor) and [Solo](../airframes/airframe_reference.md#copter_quadrotor_x_3dr_solo), Hex (Typhoon H480), [Generic quad delta VTOL](../airframes/airframe_reference.md#vtol_standard_vtol_generic_quad_delta_vtol), Tailsitter, Plane, Rover, Submarine/UUV.

> **Warning** Gazebo is often used with [ROS](../ros/README.md), a toolkit/offboard API for automating vehicle control. If you plan to use PX4 with ROS you **should follow the** [ROS Instructions](../simulation/ros_interface.md) to install both ROS and Gazebo (and thereby avoid installation conflicts).

{% youtube %}https://www.youtube.com/watch?v=qfFF9-0k4KA&vq=hd720{% endyoutube %}

{% mermaid %} graph LR; Gazebo-->Plugin; Plugin-->MAVLink; MAVLink-->SITL; {% endmermaid %}

> **Note** See [Simulation](/simulation/README.md) for general information about simulators, the simulation environment, and simulation configuration (e.g. supported vehicles).

## Installation {#installation}

Gazebo 9 的安装在标准的环境编译已有说明。

* ** macOS：** [ Mac 上的开发环境](../setup/dev_env_mac.md)
* **Linux:** [Development Environment on Ubuntu LTS / Debian Linux > Gazebo, JMAVSim and NuttX (Pixhawk) Targets](../setup/dev_env_linux_ubuntu.md#sim_nuttx)
* ** Windows：**还不支持。

其他安装说明可在 [gazebosim.org](http://gazebosim.org/tutorials?cat=guided_b&tut=guided_b1) 上找到。

## 运行仿真

您可以通过启动 PX4 SITL和 Gazebo 来运行模拟，并加载机身配置（支持多旋翼飞机，飞机，VTOL，光流和多机仿真）。

最简单的方法是在 PX4 * Firmware *存储库的根目录中打开一个终端，并为目标调用` make `，如以下部分所示。

> **Tip** 你可以使用 [ instructions below ](#start_px4_sim_separately) 来保持 Gazebo 在后台运行然后只重启 PX4。 这样比同时重启两者要快一些。 You can also run the simulation in [Headless Mode](#headless) which does not start the Gazebo UI (this uses fewer resources and is much faster).

<span></span>

> **Tip** 使用命令 `make px4_sitl list_vmd_make_targets` 获取所有支持的平台（你还可以过滤掉以 `gazebo_` 开头的平台）。

<span></span>

> **Note** The [Installing Files and Code](../setup/dev_env.md) guide is a useful reference if there are build errors.

### 四旋翼

```sh
cd ~/src/Firmware
make px4_sitl gazebo
```

### 带光流的四旋翼

```sh
make px4_sitl gazebo_iris_opt_flow
```

### 3DR Solo (Quadrotor) {#3dr-solo}

```sh
make px4_sitl gazebo_solo
```

![3DR Solo in Gazebo](../../assets/gazebo/solo.png)

### Typhoon H480 (Hexrotor) {#typhoon_h480}

    make px4_sitl gazebo_typhoon_h480
    

![Typhoon H480 in Gazebo](../../assets/gazebo/typhoon.jpg)

> **Note** This target also supports [video streaming simulation](#video).

### Standard Plane

```sh
make px4_sitl gazebo_plane
```

![Plane in Gazebo](../../assets/gazebo/plane.png)

### Standard VTOL

```sh
make px4_sitl gazebo_standard_vtol
```

![Standard VTOL in Gazebo](../../assets/gazebo/standard_vtol.png)

### Tailsitter VTOL

```sh
make px4_sitl gazebo_tailsitter
```

![Tailsitter VTOL in Gazebo](../../assets/gazebo/tailsitter.png)

### Ackerman vehicle (UGV/Rover) {#ugv}

```sh
make px4_sitl gazebo_rover
```

![Rover in Gazebo](../../assets/gazebo/rover.png)

### HippoCampus TUHH (UUV: Unmanned Underwater Vehicle) {#uuv}

```sh
make px4_sitl gazebo_uuv_hippocampus
```

![Submarine/UUV](../../assets/gazebo/hippocampus.png)

### Boat (USV: Unmanned Surface Vehicle) {#usv}

```sh
make px4_sitl gazebo_boat
```

![Boat/USV](../../assets/gazebo/boat.png)

## Taking it to the Sky

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

## Usage/Configuration Options

### Headless Mode {#headless}

Gazebo can be run in a *headless* mode in which the Gazebo UI is not launched. This starts up more quickly and uses less system resources (i.e. it is a more "lightweight" way to run the simulation).

Simply prefix the normal `make` command with `HEADLESS=1` as shown:

```bash
HEADLESS=1 make px4_sitl gazebo_plane
```

### Set Custom Takeoff Location {#custom_takeoff_location}

The default takeoff location in SITL Gazebo can be overridden using environment variables.

The variables to set are: `PX4_HOME_LAT`, `PX4_HOME_LON`, and `PX4_HOME_ALT`.

For example:

    export PX4_HOME_LAT=28.452386
    export PX4_HOME_LON=-13.867138
    export PX4_HOME_ALT=28.5
    make px4_sitl gazebo
    

### Change Simulation Speed

The simulation speed can be increased or decreased with respect to realtime using the environment variable `PX4_SIM_SPEED_FACTOR`.

    export PX4_SIM_SPEED_FACTOR=2
    make px4_sitl_default gazebo
    

For more information see: [Simulation > Run Simulation Faster than Realtime](../simulation/README.md#simulation_speed).

### Using a Joystick

Joystick and thumb-joystick support are supported through *QGroundControl* ([setup instructions here](../simulation/README.md#joystickgamepad-integration)).

### Improving Distance Sensor Performance

The current default world is [PX4/sitl_gazebo/worlds/**iris.world**](https://github.com/PX4/sitl_gazebo/tree/master/worlds)), which uses a heightmap as ground.

This can cause difficulty when using a distance sensor. If there are unexpected results we recommend you change the model in **iris.model** from `uneven_ground` to `asphalt_plane`.

### Simulating GPS Noise {#gps_noise}

Gazebo can simulate GPS noise that is similar to that typically found in real systems (otherwise reported GPS values will be noise-free/perfect). This is useful when working on applications that might be impacted by GPS noise - e.g. precision positioning.

GPS noise is enabled if the target vehicle's SDF file contains a value for the `gpsNoise` element (i.e. it has the line: `<gpsNoise>true</gpsNoise>`). It is enabled by default in many vehicle SDF files: **solo.sdf**, **iris.sdf**, **standard_vtol.sdf**, **delta_wing.sdf**, **plane.sdf**, **typhoon_h480**, **tailsitter.sdf**.

To enable/disable GPS noise:

1. 构建任何 gazebo 目标以生成 SDF 文件（适用于所有机型）。 例如： ```make px4_sitl gazebo_iris``` >**Tip**在后续版本中不会覆盖 SDF 文件。
2. 打开目标车辆的 SDF 文件（例如**./Tools/sitl_gazebo/models/iris/iris.sdf **）。
3. 搜索 `gpsNoise` 元素： 
        xml
        <plugin name='gps_plugin' filename='libgazebo_gps_plugin.so'>
         <robotNamespace/>
         <gpsNoise>true</gpsNoise>
        </plugin>
    
    * 如果存在，则启用 GPS。 您可以通过删除以下行来禁用它：`<gpsNoise> true </gpsNoise>`
    * 如果未预设，则禁用 GPS 。 您可以通过将` gpsNoise `元素添加到` gps_plugin `部分来启用它（如上所示）。

The next time you build/restart Gazebo it will use the new GPS noise setting.

## Starting Gazebo and PX4 Separately {#start_px4_sim_separately}

For extended development sessions it might be more convenient to start Gazebo and PX4 separately or even from within an IDE.

In addition to the existing cmake targets that run `sitl_run.sh` with parameters for px4 to load the correct model it creates a launcher targets named `px4_<mode>` that is a thin wrapper around original sitl px4 app. This thin wrapper simply embeds app arguments like current working directories and the path to the model file.

To start Gazebo and PX4 separately:

* Run gazebo (or any other sim) server and client viewers via the terminal specifing an `_ide` variant: 
        sh
        make px4_sitl gazebo___ide or 
    
        sh
        make px4_sitl gazebo_iris_ide

* 在 IDE 中选择要调试的` px4_ <mode> `目标（例如` px4_iris `）
* 直接从 IDE 启动调试会话

This approach significantly reduces the debug cycle time because simulator (e.g. Gazebo) is always running in background and you only re-run the px4 process which is very light.

## Simulated Survey Camera

The *Gazebo* survey camera simulates a [MAVLink camera](https://mavlink.io/en/services/camera.html) that captures geotagged JPEG images and sends camera capture information to a connected ground station. It can be used to test camera capture, in particular within survey missions.

The camera emits the [CAMERA_IMAGE_CAPTURED](https://mavlink.io/en/messages/common.html#CAMERA_IMAGE_CAPTURED) message every time an image is captured. The captured images are saved to: **Firmware/build/px4_sitle_default/tmp/frames/DSC_n_.jpg** (where *n* starts as 00000 and is iterated by one on each capture).

To simulate a plane with this camera:

    make px4_sitl_default gazebo_plane_cam
    

> **Note** The camera also supports/responds to the following MAVLink commands: [MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS](https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS), [MAV_CMD_REQUEST_STORAGE_INFORMATION](https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_STORAGE_INFORMATION), [MAV_CMD_REQUEST_CAMERA_SETTINGS](https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_CAMERA_SETTINGS), [MAV_CMD_REQUEST_CAMERA_INFORMATION](https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_CAMERA_INFORMATION), [MAV_CMD_RESET_CAMERA_SETTINGS](https://mavlink.io/en/messages/common.html#MAV_CMD_RESET_CAMERA_SETTINGS), [MAV_CMD_STORAGE_FORMAT](https://mavlink.io/en/messages/common.html#MAV_CMD_STORAGE_FORMAT), [MAV_CMD_SET_CAMERA_ZOOM](https://mavlink.io/en/messages/common.html#MAV_CMD_SET_CAMERA_ZOOM), [MAV_CMD_IMAGE_START_CAPTURE](https://mavlink.io/en/messages/common.html#MAV_CMD_IMAGE_START_CAPTURE), [MAV_CMD_IMAGE_STOP_CAPTURE](https://mavlink.io/en/messages/common.html#MAV_CMD_IMAGE_STOP_CAPTURE).

<span></span>

> **Note** The simulated camera is implemented in [PX4/sitl_gazebo/src/gazebo_geotagged_images_plugin.cpp](https://github.com/PX4/sitl_gazebo/blob/master/src/gazebo_geotagged_images_plugin.cpp).

## Video Streaming {#video}

PX4 SITL for Gazebo supports UDP video streaming from a Gazebo camera sensor attached to a vehicle model. When streaming is enabled, you can connect to this stream from *QGroundControl* (on UDP port 5600) and view video of the Gazebo environment from the simulated vehicle - just as you would from a real camera. The video is streamed using a *gstreamer* pipeline and can be enabled/disabled using a button in the Gazebo UI.

The Gazebo camera sensor is supported/enabled on the following frames:

* [Typhoon H480](#typhoon_h480)

### Prerequisites

*Gstreamer 1.0* is required for video streaming. The required dependencies should already have been [installed when you set up Gazebo](#installation) (they are included in the standard PX4 installation scripts/instructions for macOS and Ubuntu Linux).

> **Note** FYI only, the dependencies include: `gstreamer1.0-plugins-base`, g`streamer1.0-plugins-good`, `gstreamer1.0-plugins-bad`, `gstreamer1.0-plugins-ugly`, `libgstreamer-plugins-base1.0-dev`.

### Start/Stop Video Streaming

Video streaming is automatically started when supported by the target vehicle. For example, to start streaming video on the Typhoon H480:

    make px4_sitl gazebo_typhoon_h480
    

Streaming can be paused/restarted using the Gazebo UI *Video ON/OFF* button..

![Video ON/OFF button](../../assets/gazebo/sitl_video_stream.png)

### How to View Gazebo Video

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

## Extending and Customizing

To extend or customize the simulation interface, edit the files in the `Tools/sitl_gazebo` folder. The code is available on the [sitl_gazebo repository](https://github.com/px4/sitl_gazebo) on Github.

> **Note** The build system enforces the correct GIT submodules, including the simulator. It will not overwrite changes in files in the directory.

## Further Information

* [ROS with Gazebo Simulation](../simulation/ros_interface.md)
* [Gazebo Octomap](../simulation/gazebo_octomap.md)