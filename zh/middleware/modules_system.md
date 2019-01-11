# 模块参考：系统

## dataman

源码： [modules/dataman](https://github.com/PX4/Firmware/tree/master/src/modules/dataman)

### 描述

Module to provide persistent storage for the rest of the system in form of a simple database through a C API. Multiple backends are supported:

- a file (eg. on the SD card)
- FLASH (if the board supports it)
- FRAM
- RAM (this is obviously not persistent)

It is used to store structured data of different types: mission waypoints, mission state and geofence polygons. Each type has a specific type and a fixed maximum amount of storage items, so that fast random access is possible.

### 实现

Reading and writing a single item is always atomic. If multiple items need to be read/modified atomically, there is an additional lock per item type via `dm_lock`.

**DM_KEY_FENCE_POINTS** and **DM_KEY_SAFE_POINTS** items: the first data element is a `mission_stats_entry_s` struct, which stores the number of items for these types. These items are always updated atomically in one transaction (from the mavlink mission manager). During that time, navigator will try to acquire the geofence item lock, fail, and will not check for geofence violations.

### 用法 {#dataman_usage}

    dataman <command> [arguments...]
     Commands:
       start
         [-f <val>]  Storage file
                     values: <file>
         [-r]        Use RAM backend (NOT persistent)
         [-i]        Use FLASH backend
    
     The options -f, -r and -i are mutually exclusive. If nothing is specified, a
     file 'dataman' is used
    
       poweronrestart Restart dataman (on power on)
    
       inflightrestart Restart dataman (in flight)
    
       stop
    
       status        print status info
    

## heater

源码：[drivers/heater](https://github.com/PX4/Firmware/tree/master/src/drivers/heater)

### 描述

Background process running periodically on the LP work queue to regulate IMU temperature at a setpoint.

This task can be started at boot from the startup scripts by setting SENS_EN_THERMAL or via CLI.

### 用法 {#heater_usage}

    heater <command> [arguments...]
     Commands:
       controller_period Reports the heater driver cycle period value, (us), and
                     sets it if supplied an argument.
    
       integrator    Sets the integrator gain value if supplied an argument and
                     reports the current value.
    
       proportional  Sets the proportional gain value if supplied an argument and
                     reports the current value.
    
       sensor_id     Reports the current IMU the heater is temperature controlling.
    
       setpoint      Reports the current IMU temperature.
    
       start         Starts the IMU heater driver as a background task
    
       status        Reports the current IMU temperature, temperature setpoint, and
                     heater on/off status.
    
       stop          Stops the IMU heater driver.
    
       temp          Reports the current IMU temperature.
    
       stop
    
       status        print status info
    

## land_detector

源码：[modules/land_detector](https://github.com/PX4/Firmware/tree/master/src/modules/land_detector)

### 描述

Module to detect the freefall and landed state of the vehicle, and publishing the `vehicle_land_detected` topic. Each vehicle type (multirotor, fixedwing, vtol, ...) provides its own algorithm, taking into account various states, such as commanded thrust, arming state and vehicle motion.

### 实现

Every type is implemented in its own class with a common base class. The base class maintains a state (landed, maybe_landed, ground_contact). Each possible state is implemented in the derived classes. A hysteresis and a fixed priority of each internal state determines the actual land_detector state.

#### Multicopter Land Detector

**ground_contact**: thrust setpoint and velocity in z-direction must be below a defined threshold for time GROUND_CONTACT_TRIGGER_TIME_US. When ground_contact is detected, the position controller turns off the thrust setpoint in body x and y.

**maybe_landed**: it requires ground_contact together with a tighter thrust setpoint threshold and no velocity in the horizontal direction. The trigger time is defined by MAYBE_LAND_TRIGGER_TIME. When maybe_landed is detected, the position controller sets the thrust setpoint to zero.

**landed**: it requires maybe_landed to be true for time LAND_DETECTOR_TRIGGER_TIME_US.

The module runs periodically on the HP work queue.

### 用法 {#land_detector_usage}

    land_detector <command> [arguments...]
     Commands:
       start         启动后台任务
         fixedwing|multicopter|vtol|ugv 选择飞机类型
    
       stop
    
       status        打印状态信息
    

## load_mon

源码：[modules/load_mon](https://github.com/PX4/Firmware/tree/master/src/modules/load_mon)

### 描述

Background process running periodically with 1 Hz on the LP work queue to calculate the CPU load and RAM usage and publish the `cpuload` topic.

On NuttX it also checks the stack usage of each process and if it falls below 300 bytes, a warning is output, which will also appear in the log file.

### 用法 {#load_mon_usage}

    load_mon <command> [arguments...]
     Commands:
       start         启动后台任务
    
       stop
    
       status        打印状态信息
    

## logger

源码：[modules/logger](https://github.com/PX4/Firmware/tree/master/src/modules/logger)

### 描述

System logger which logs a configurable set of uORB topics and system printf messages (`PX4_WARN` and `PX4_ERR`) to ULog files. These can be used for system and flight performance evaluation, tuning, replay and crash analysis.

It supports 2 backends:

- Files: write ULog files to the file system (SD card)
- MAVLink: stream ULog data via MAVLink to a client (the client must support this)

Both backends can be enabled and used at the same time.

The file backend supports 2 types of log files: full (the normal log) and a mission log. The mission log is a reduced ulog file and can be used for example for geotagging or vehicle management. It can be enabled and configured via SDLOG_MISSION parameter. The normal log is always a superset of the mission log.

### 实现

The implementation uses two threads:

- The main thread, running at a fixed rate (or polling on a topic if started with -p) and checking for data updates
- The writer thread, writing data to the file

In between there is a write buffer with configurable size (and another fixed-size buffer for the mission log). It should be large to avoid dropouts.

### 示例

Typical usage to start logging immediately:

    logger start -e -t
    

Or if already running:

    logger on
    

### 用法 {#logger_usage}

    logger <command> [arguments...]
     Commands:
       start
         [-m <val>]  Backend mode
                     values: file|mavlink|all, default: all
         [-e]        Enable logging right after start until disarm (otherwise only
                     when armed)
         [-f]        Log until shutdown (implies -e)
         [-t]        Use date/time for naming log directories and files
         [-r <val>]  Log rate in Hz, 0 means unlimited rate
                     default: 280
         [-b <val>]  Log buffer size in KiB
                     default: 12
         [-q <val>]  uORB queue size for mavlink mode
                     default: 14
         [-p <val>]  Poll on a topic instead of running with fixed rate (Log rate
                     and topic intervals are ignored if this is set)
                     values: <topic_name>
    
       on            start logging now, override arming (logger must be running)
    
       off           stop logging now, override arming (logger must be running)
    
       stop
    
       status        print status info
    

## replay

源码： [modules/replay](https://github.com/PX4/Firmware/tree/master/src/modules/replay)

### 描述

This module is used to replay ULog files.

There are 2 environment variables used for configuration: `replay`, which must be set to an ULog file name - it's the log file to be replayed. The second is the mode, specified via `replay_mode`:

- `replay_mode=ekf2`: specific EKF2 replay mode. It can only be used with the ekf2 module, but allows the replay to run as fast as possible.
- Generic otherwise: this can be used to replay any module(s), but the replay will be done with the same speed as the log was recorded.

The module is typically used together with uORB publisher rules, to specify which messages should be replayed. The replay module will just publish all messages that are found in the log. It also applies the parameters from the log.

The replay procedure is documented on the [System-wide Replay](https://dev.px4.io/en/debug/system_wide_replay.html) page.

### 用法 {#replay_usage}

    replay <command> [arguments...]
     Commands:
       start         Start replay, using log file from ENV variable 'replay'
    
       trystart      Same as 'start', but silently exit if no log file given
    
       tryapplyparams Try to apply the parameters from the log file
    
       stop
    
       status        print status info
    

## send_event

源码： [modules/events](https://github.com/PX4/Firmware/tree/master/src/modules/events)

### 描述

Background process running periodically on the LP work queue to perform housekeeping tasks. It is currently only responsible for temperature calibration and tone alarm on RC Loss.

The tasks can be started via CLI or uORB topics (vehicle_command from MAVLink, etc.).

### 用法 {#send_event_usage}

    send_event <command> [arguments...]
     Commands:
       start         Start the background task
    
       temperature_calibration Run temperature calibration process
         [-g]        calibrate the gyro
         [-a]        calibrate the accel
         [-b]        calibrate the baro (if none of these is given, all will be
                     calibrated)
    
       stop
    
       status        print status info
    

## sensors

源码： [modules/sensors](https://github.com/PX4/Firmware/tree/master/src/modules/sensors)

### 描述

The sensors module is central to the whole system. It takes low-level output from drivers, turns it into a more usable form, and publishes it for the rest of the system.

The provided functionality includes:

- Read the output from the sensor drivers (`sensor_gyro`, etc.). If there are multiple of the same type, do voting and failover handling. Then apply the board rotation and temperature calibration (if enabled). And finally publish the data; one of the topics is `sensor_combined`, used by many parts of the system.
- Do RC channel mapping: read the raw input channels (`input_rc`), then apply the calibration, map the RC channels to the configured channels & mode switches, low-pass filter, and then publish as `rc_channels` and `manual_control_setpoint`.
- Read the output from the ADC driver (via ioctl interface) and publish `battery_status`.
- Make sure the sensor drivers get the updated calibration parameters (scale & offset) when the parameters change or on startup. The sensor drivers use the ioctl interface for parameter updates. For this to work properly, the sensor drivers must already be running when `sensors` is started.
- Do preflight sensor consistency checks and publish the `sensor_preflight` topic.

### 实现

It runs in its own thread and polls on the currently selected gyro topic.

### 用法 {#sensors_usage}

    sensors <command> [arguments...]
     Commands:
       start
         [-h]        在 HIL 模式下启动
    
       stop
    
       status        打印状态信息
    

## tune_control

源码：[systemcmds/tune_control](https://github.com/PX4/Firmware/tree/master/src/systemcmds/tune_control)

### 描述

Command-line tool to control & test the (external) tunes.

Tunes are used to provide audible notification and warnings (e.g. when the system arms, gets position lock, etc.). The tool requires that a driver is running that can handle the tune_control uorb topic.

Information about the tune format and predefined system tunes can be found here: https://github.com/PX4/Firmware/blob/master/src/lib/tunes/tune_definition.desc

### 示例

Play system tune #2:

    tune_control play -t 2
    

### 用法 {#tune_control_usage}

    tune_control <command> [arguments...]
     Commands:
       play          Play system tune, tone, or melody
         [-t <val>]  Play predefined system tune
                     default: 1
         [-f <val>]  Frequency of tone in Hz (0-22kHz)
                     default: 0
         [-d <val>]  Duration of tone in us
                     default: 1
         [-s <val>]  Strength of tone (0-100)
                     default: 40
         [-m <val>]  Melody in string form
                     values: <string> - e.g. "MFT200e8a8a"
    
       libtest       Test library
    
       stop          Stop playback (use for repeated tunes)