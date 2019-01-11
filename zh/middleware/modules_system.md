# 模块参考：系统

## dataman

源码： [modules/dataman](https://github.com/PX4/Firmware/tree/master/src/modules/dataman)

### 描述

该模块通过基于 C 语言的 API 以简单数据库的形式为系统的其它部分提供持续性存储功能。 支持多种后端：

- 一个文件 （比如，在 SD 卡上）
- FLASH 内存（如果飞控板支持的话）
- FRAM
- 内存 RAM (显然这种方式不是持续的)

该模块用来存储不同类型的结构化数据：任务航点、人物状态和地理围栏多边形。 每种类型的数据都有一个特定的类型和一个固定的最大存储条目的数量，因此可以实现对数据的快速随机访问。

### 实现

读取和写入单个项目总是原子的。 如果需要对多个条目进行原子读取/修改，模块会使用 `dm_lock` 对每个类型的条目添加一个额外的锁定。

**DM_KEY_FENCE_POINTS** 和 **DM_KEY_SAFE_POINTS** 条目：第一个数据元素是一个 `mission_stats_entry_s` 结构体，存储着这些类型的项目的条目数量。 这些项目在每个业务中都会进行原子更新 (从 mavlink 任务管理器)。 在此期间，navigator 会尝试获取地理围栏条目的锁定，如果失败则不会检查是否超越了地理围栏。

### 用法 {#dataman_usage}

    dataman &lt;command&gt; [arguments...]
     Commands:
       start
         [-f &lt;val&gt;]  存储文件
                     取值: &lt;file&gt;
         [-r]        使用 RAM 后端 (非持续)
         [-i]        使用 FLASH 后端
    
     -f, -r 和-i 选项是互斥的。 如果未指定后端，那么默认使用文件 'dataman' 
    
       poweronrestart 重启 dataman (处于开机 power on 状态时)
    
       inflightrestart 重启 dataman (处于飞行状态时)
    
       stop
    
       status        打印状态信息
    

## heater

源码：[drivers/heater](https://github.com/PX4/Firmware/tree/master/src/drivers/heater)

### 描述

此模块将以后台进程形式在 LP 工作列队中周期性运行，以实现将 IMU 的温度调节至一个设定值。

此任务可以在启动脚本通过设置 SENS_EN_THERMAL 运行或者直接通过 CLI 命令行启动。

### 用法 {#heater_usage}

    heater &lt;command&gt; [arguments...]
     Commands:
       controller_period 报告 heater 驱动周期间隔数值，（us），若
                     提供了参数值则设置 heater 驱动的周期间隔。
    
       integrator    报告当前积分增益的值，如果
                     提供了参数值则设置积分增益。
    
       proportional  报告当前比例增益的值，如果
                     提供了参数值则设置比例增益。
    
       sensor_id     报告 heater 驱动当前正在进行温度控制的 IMU 传感器的 ID。
    
       setpoint      报告当前 IMU 温度。
    
       start         运行 IMU heater 驱动的后台任务
    
       status        报告当前 IMU 温度，温度设定值和 heater 的 on/off 状态。
    
       stop          停止 IMU heater 驱动。
    
       temp          报告当前 IMU 温度。
    
       stop
    
       status        打印状态信息
    

## land_detector

源码：[modules/land_detector](https://github.com/PX4/Firmware/tree/master/src/modules/land_detector)

### 描述

该模块会检测无人机的自由落体状态和触地状态，并将相关数据从 `vehicle_land_detected` 主题中发布出去。 每一个类型的无人机（多旋翼， vtol，...）都有各自的检测算法，该算法会考虑无人机的多种状态，例如指令推力、解锁状态、飞机运动状态等。

### 实现

每一类都是基于一个公共的基类在各自独有的类中完成模块的实现。 基类中包含了一个状态量 (landed, maybe_landed, ground_contact）。 每一个可能的状态都在衍生出的子类中进行了实现。 每个内部状态的迟滞和固定优先级共同决定着实际的 land_detector 的状态。

#### 多旋翼的 Land Detector

**ground_contact**: 在 GROUND_CONTACT_TRIGGER_TIME_US 时间内推力设定值和飞机 z 方向的速度必须在低于一个预设的阈值， 当检测到 ground_contact 状态时，位置控制器将关闭机体 x 方向和 y 方向上的推力设定值。

**maybe_landed**: 该状态除了要求飞机处于 ground_contact 状态外，还要求满足一个更严格的推理设定值，且飞机在水平方向上没有速度。 触发时间由变量 MAYBE_LAND_TRIGGER_TIME 定义。 当检测到 maybe_landed 状态时，位置控制器会将推理设定值设置为零。

**landed**: 它要求在 LAND_DETECTOR_TRIGGER_TIME_US 时间内 maybe_landed 状态为真。

该模块在 HP 工作队列中周期性运行。

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

模块在 HP 工作队列中以 1 Hz 频率周期性计算 CPU 负载、 RAM 使用情况，并将结果发布到 `cpuload` 主题。

在 NuttX 平台上该模块还会检查每个进程的栈堆使用情况，如果它低于 300 字节那么模块会输出一个警告，该警告会出现在日志文件中。

### 用法 {#load_mon_usage}

    load_mon <command> [arguments...]
     Commands:
       start         启动后台任务
    
       stop
    
       status        打印状态信息
    

## logger

源码：[modules/logger](https://github.com/PX4/Firmware/tree/master/src/modules/logger)

### 描述

系统 logger 会记录一组可配置的 uORB 主题及系统 printf 消息（`PX4_WARN` 和 `PX4_ERR`）到 ULog 文件中。 该日志文件可用于系统性能和飞行表现的评估、调参、回放和事故分析。

该模块支持 2 个后端：

- 文件：写入 ULog 文件到文件系统中（SD 卡）
- MAVLink: 通过 MAVLink 将 ULog 数据流传输到客户端上（需要客户端支持此方式）

两种后端可同时启用。

文件后端支持 2 种类型的日志文件：完整日志（标准日志）和任务日志。 任务日志是一个精简的 ulog 文件，可用于地理标记或者无人机管理等用途。 可使用 SDLOG_MISSION 参数来启用和配置记录任务日志。 标准日志始终是任务日志的父集。

### 实现

模块的实现使用了两个线程：

- 主进程以固定速率运行（如果以 -p 参数启动则会轮询一个主题），并检查数据的更新。
- 写入线程，将数据写入文件中、

两个线程中间有一个可配置大小的写缓冲区（有另一个固定大小的缓冲区用于任务日志的写入）。 缓冲区应大到可以避免出现数据溢出。

### 示例

立刻开始记录日志的典型用法：

    logger start -e -t
    

或者当模块已经在运行时：

    logger on
    

### 用法 {#logger_usage}

    logger &lt;command&gt; [arguments...]
     Commands:
       start
         [-m &lt;val&gt;]  后端模式
                     取值： file|mavlink|all, 默认值： all
         [-e]        完成启动后立刻启用日志记录，直至锁定飞机(否则仅当处于解锁状态才会开始日志记录)
         [-f]        进行日志记录直到关机 (implies -e)
         [-t]        使用日期/时间来命名日志文件夹和文件
         [-r &lt;val&gt;]  日志记录速率，单位 Hz, 0 表示不限制速率
                     默认值： 280
         [-b &lt;val&gt;]  日志缓冲区大小，单位 KiB
                     默认值： 12
         [-q &lt;val&gt;]  mavlink 模式下 uORB 队列的大小
                     默认值：14
         [-p &lt;val&gt;]  轮询一个主题而不是以一个固定速率去检查 （进行了此项设定后日志记录速率和主题周期会被忽略）
                     取值： &lt;topic_name&gt;
    
       on            立刻开始日志记录，覆盖解锁飞机指令 (logger 必须处于运行状态)
    
       off           立刻停止日志记录，覆盖解锁飞机指令 (logger 必须处于运行状态)
    
       stop
    
       status        打印状态信息
    

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