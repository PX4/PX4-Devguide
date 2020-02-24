# Gazebo Simulation

<!-- Check if updates required - ie that following are fixed:

- Gazebo8 not supported and should be: https://github.com/PX4/sitl_gazebo/pull/118#pullrequestreview-86032497
- Video functionality disabled by default (should be enabled): https://github.com/PX4/Devguide/pull/418#pullrequestreview-86789154
- Find out actual gstreamer dependencies and update both this doc and build scripts - should be done when the camera plugin is a default.
-->

[Gazebo](http://gazebosim.org) is a powerful 3D simulation environment for autonomous robots that is particularly suitable for testing object-avoidance and computer vision. This page describes its use with SITL and a single vehicle. Gazebo can also be used with [HITL](../simulation/hitl.md) and for [multi-vehicle simulation](../simulation/multi-vehicle-simulation.md).

**Supported Vehicles:** Quad ([Iris](../airframes/airframe_reference.md#copter_quadrotor_wide_3dr_iris_quadrotor) and [Solo](../airframes/airframe_reference.md#copter_quadrotor_x_3dr_solo), Hex (Typhoon H480), [Generic quad delta VTOL](../airframes/airframe_reference.md#vtol_standard_vtol_generic_quad_delta_vtol), Tailsitter, Plane, Rover, Submarine/UUV.

> **Tip** Gazebo is often used with [ROS](../ros/README.md), a toolkit/offboard API for automating vehicle control. If you plan to use PX4 with ROS you should instead [follow the instructions here](../simulation/ros_interface.md) to install Gazebo as part of ROS!

{% youtube %}https://www.youtube.com/watch?v=qfFF9-0k4KA&vq=hd720{% endyoutube %}

{% mermaid %} graph LR; Gazebo-->Plugin; Plugin-->MAVLink; MAVLink-->SITL; {% endmermaid %}

> **Note** See [Simulation](/simulation/README.md) for general information about simulators, the simulation environment and available simulation configuration (e.g. supported vehicles).

## Installation

Gazebo 9 setup is included in our standard build instructions:

* **macOS:** [Development Environment on Mac](../setup/dev_env_mac.md)
* **Linux:** [Development Environment on Linux (Ubuntu 16.04) > jMAVSim/Gazebo Simulation](../setup/dev_env_linux_ubuntu.md#sim_nuttx)
* **Windows:** Not supported.

Additional installation instructions can be found on [gazebosim.org](http://gazebosim.org/tutorials?cat=guided_b&tut=guided_b1).

## Running the Simulation

You can run a simulation by starting PX4 SITL and gazebo with the airframe configuration to load (multicopters, planes, VTOL, optical flow and multi-vehicle simulations are supported).

The easiest way to do this is to open a terminal in the root directory of the PX4 *Firmware* repository and call `make` for the targets as shown in the following sections.

> **Tip** You can use the [instructions below](#start_px4_sim_separately) to keep Gazebo running and only re-launch PX4. This is quicker than restarting both.

<span></span>

> **Tip** For the full list of build targets run `make px4_sitl list_vmd_make_targets` (and filter on those that start with `gazebo_`).

### Quadrotor

```sh
cd ~/src/Firmware
make px4_sitl gazebo
```

### Quadrotor with Optical Flow

```sh
make px4_sitl gazebo_iris_opt_flow
```

### 3DR Solo

```sh
make px4_sitl gazebo_solo
```

![3DR Solo in Gazebo](../../assets/gazebo/solo.png)

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

## Change World

The current default world is the **iris.world** located in the directory [worlds](https://github.com/PX4/sitl_gazebo/tree/b59e6e78e42d50f70224d1d0e506825590754d64/worlds). The default surrounding in the **iris.world** uses a heightmap as ground. This ground can cause difficulty when using a distance sensor. If there are unexpected results with that heightmap, we recommend you change the model in **iris.model** from `uneven_ground` to `asphalt_plane`.

## Taking it to the Sky

> **Note** Please refer to the [Installing Files and Code](../setup/dev_env.md) guide if you run into any errors.

This will bring up the PX4 shell:

```sh
[init] shell id: 140735313310464
[init] task name: px4

______  __   __    ___
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.


pxh>
```

> **Note** Right-clicking the quadrotor model allows to enable follow mode from the context menu, which is handy to keep it in view.

![Gazebo UI](../../assets/simulation/gazebo.png)

The system will print the home position once it finished intializing (`telem> home: 55.7533950, 37.6254270, -0.00`). You can bring it into the air by typing:

```sh
pxh> commander takeoff
```

## Usage/Configuration Options

### Headless Mode

Gazebo can be run in a *headless* mode in which the Gazebo UI is not launched. This starts up more quickly and uses less system resources (i.e. it is a more "lightweight" way to run the simulation).

Simply prefix the normal *make* command with `HEADLESS=1` as shown:

```bash
HEADLESS=1 make px4_sitl gazebo_plane
```

### Set Custom Takeoff Location

The default takeoff location in SITL Gazebo can be overridden using environment variables.

The variables to set are: `PX4_HOME_LAT`, `PX4_HOME_LON`, and `PX4_HOME_ALT`.

As an example:

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

### Simulating GPS Noise

Gazebo can simulate GPS noise that is similar to that typically found in real systems (otherwise reported GPS values will be noise-free/perfect). This is useful when working on applications that might be impacted by GPS noise - e.g. precision positioning.

GPS noise is enabled if the target vehicle's SDF file contains a value for the `gpsNoise` element (i.e. it has the line: `<gpsNoise>true</gpsNoise>`). It is enabled by default in many vehicle SDF files: **solo.sdf**, **iris.sdf**, **standard_vtol.sdf**, **delta_wing.sdf**, **plane.sdf**, **typhoon_h480**, **tailsitter.sdf**.

To enable/disable GPS noise:

1. Build any gazebo target in order to generate SDF files (for all vehicles). For example: ```make px4_sitl gazebo_iris``` > **Tip** The SDF files are not overwritten on subsequent builds. 
2. Open the SDF file for your target vehicle (e.g. **./Tools/sitl_gazebo/models/iris/iris.sdf**).
3. Search for the `gpsNoise` element: 
        xml
        <plugin name='gps_plugin' filename='libgazebo_gps_plugin.so'>
         <robotNamespace/>
         <gpsNoise>true</gpsNoise>
        </plugin>
    
    * If it is present, GPS is enabled. You can disable it by deleting the line: `<gpsNoise>true</gpsNoise>`
    * If it is not preset GPS is disabled. You can enable it by adding the `gpsNoise` element to the `gps_plugin` section (as shown above).

The next time you build/restart Gazebo it will use the new GPS noise setting.

## Starting Gazebo and PX4 Separately {#start_px4_sim_separately}

For extended development sessions it might be more convenient to start Gazebo and PX4 separately or even from within an IDE.

In addition to the existing cmake targets that run `sitl_run.sh` with parameters for px4 to load the correct model it creates a launcher targets named `px4_<mode>` that is a thin wrapper around original sitl px4 app. This thin wrapper simply embeds app arguments like current working directories and the path to the model file.

To start Gazebo and PX4 separately:

* Run gazebo (or any other sim) server and client viewers via the terminal: ```make px4_sitl gazebo_none_ide```
* In your IDE select `px4_<mode>` target you want to debug (e.g. `px4_iris`)
* Start the debug session directly from IDE

This approach significantly reduces the debug cycle time because simulator (e.g. gazebo) is always running in background and you only re-run the px4 process which is very light.

## Video Streaming

PX4 SITL for Gazebo supports UDP video streaming from a Gazebo camera sensor attached to a vehicle model. You can connect to this stream from *QGroundControl* (on UDP port 5600) and view video of the Gazebo environment from the simulated vehicle - just as you would from a real camera. The video is streamed using a *gstreamer* pipeline.

> **Note** Video streaming from Gazebo and the Gazebo widget to turn streaming on/off are not enabled by default. This article explains how to enable them. In the near future we expect these features to be enabled by default.

### Prerequisites

Ubuntu: Install *Gstreamer 1.0* and its dependencies:

    sudo apt-get install $(apt-cache --names-only search ^gstreamer1.0-* | awk '{ print $1 }' | grep -v gstreamer1.0-hybris) -y
    

Mac OS:

    brew install gstreamer gst-libav gst-plugins-base gst-plugins-good gst-plugins-bad gst-plugins-ugly
    

### How to View Gazebo Video

The easiest way to view the SITL/Gazebo camera video stream is in *QGroundControl*. Simply open **Settings > General** and set **Video Source** to *UDP Video Stream* and **UDP Port** to *5600*:

![QGC Video Streaming Settings for Gazebo](../../assets/simulation/qgc_gazebo_video_stream_udp.png)

The video from Gazebo should then display in *QGroundControl* just as it would from a real camera.

It is also possible to view the video using the *Gstreamer Pipeline*. Simply enter the following terminal command:

    gst-launch-1.0  -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
    ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink fps-update-interval=1000 sync=false
    

### Gazebo GUI to Start/Stop Video Streaming

> **Note** This feature is supported for Gazebo version 7.

Video streaming can be enabled/disabled using the Gazebo UI *Video ON/OFF* button.

![Video ON/OFF button](../../assets/gazebo/sitl_video_stream.png)

To enable the button:

1. Open the "world" file to be modified (e.g. [&lt;Firmware>/Tools/sitl_gazebo/worlds/typhoon_h480.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/typhoon_h480.world)).
2. Within the default `world name="default"` section, add the `gui` section for the `libgazebo_video_stream_widget` (as shown below):
    
    ```xml
    <?xml version="1.0" ?>
    <sdf version="1.5">
     <world name="default">
    ```
    
    ```xml
       <gui>
         <plugin name="video_widget" filename="libgazebo_video_stream_widget.so"/>
       </gui>
    ```
    
    ```xml
    <!-- A global light source -->
    <include>
    ...
    ```
    
    > **Tip** This section present in **typhoon_h480.world** - you just need to uncomment the section.

3. Rebuild SITL:
    
        make clean
        make px4_sitl gazebo_typhoon_h480
        

## Extending and Customizing

To extend or customize the simulation interface, edit the files in the `Tools/sitl_gazebo` folder. The code is available on the [sitl_gazebo repository](https://github.com/px4/sitl_gazebo) on Github.

> **Note** The build system enforces the correct GIT submodules, including the simulator. It will not overwrite changes in files in the directory.

## Interfacing to ROS

The simulation can be [interfaced to ROS](../simulation/ros_interface.md) the same way as onboard a real vehicle.

## Further Information

* [ROS with Gazebo Simulation](../simulation/ros_interface.md)
* [Gazebo Octomap](../simulation/gazebo_octomap.md)