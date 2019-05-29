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

    logger <command> [arguments...]
     Commands:
       start
         [-m <val>]  Backend mode
                     values: file|mavlink|all, default: all
         [-x]        Enable/disable logging via Aux1 RC channel
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

此模块用于回放 ULog 文件。

共有两个需要进行配置的环境变量： `replay` ，必须被设置为 ULog 文件名 - 也就是需要进行回放的日志文件。 第二个则是通过 `replay_mode` 变量对回放模式进行设定：

- `replay_mode=ekf2`: 指定 EKF2 回放模式。 该模式只能与 ekf2 模块一起使用，但它可以让回放的运行速度尽可能的快。
- 否则为 Generic ：该模式可用于回放任何模块，但回放速度只能与日志记录的速度相同。

该模块通常与 uORB 发布者规则配合使用以指定需要进行回放的消息。 都则的话回放模块将直接发布所有在日志中找到的消息。 这也适用于在日志文件爱你中记录的各参数。

回放步骤在 [System-wide Replay](https://dev.px4.io/en/debug/system_wide_replay.html) 页面中有详细记录。

### 用法 {#replay_usage}

    replay &lt;command&gt; [arguments...]
     Commands:
       start         开始回放，使用环境变量 'replay' 中指定的日志文件
    
       trystart      与 'start' 相同，但如果未指定日志文件的话会安静地退出
    
       tryapplyparams 尝试应用日志文件中的参数
    
       stop
    
       status        打印状态信息
    

## send_event

源码： [modules/events](https://github.com/PX4/Firmware/tree/master/src/modules/events)

### 描述

此模块将以后台进程形式在 LP 工作列队中周期性运行，以执行内部管理任务。 目前它只负责校正温度和丢失 RC 信号时发出声音警报。

这些任务可以通过 CLI 命令行或者 uORB 话题（例如，来自 MAVLink 的 vehicle_command）进行启动。

### 用法 {#send_event_usage}

    send_event &lt;command&gt; [arguments...]
     Commands:
       start         开启后台任务
    
       temperature_calibration 开始温度校正程序
         [-g]        校正陀螺仪
         [-a]        校正加速度计
         [-b]        校正气压计 (如果未指定上述三者中的任意一个那么将校正全部三个传感器)
    
       stop
    
       status        打印状态信息
    

## sensors

源码： [modules/sensors](https://github.com/PX4/Firmware/tree/master/src/modules/sensors)

### 描述

Sensors 模块是整个系统的核心。 它以传感器驱动的低级别输出作为输入，将其转换为更加可用的形式并数据发布给系统的其它部分。

模块提供的功能包括：

- 读取传感器驱动的输出 (例如，`sensor_gyro` 等)。 如果存在多个同类型传感器，那个模块将进行投票和容错处理。 然后应用飞控板的旋转和温度校正（如果被启用）。 最终发布传感器数据：其中名为 `sensor_combined` 的主题被系统的许多部件所使用。
- 执行 RC 通道映射：读取通道原始输入 (`input_rc`)，应用校正并将 RC 通道映射到配置的通道 & 模式转换开关，低通滤波器，然后发布到 `rc_channels` 和 `manual_control_setpoint` 话题中。
- 从 ADC 驱动中读取输出（通过 ioctl 接口）并发布到 `battery_status` 。
- 当参数发生变化或者启动时，确保传感器驱动获得的矫正参数（缩放因子 & 偏移量）是最新的。 传感器驱动使用 ioctl 接口获取参数更新。 为了使这一功能正常运行，当 `sensors` 模块启动时传感器驱动必须已经处于运行状态。
- 执行起飞前传感器一致性检查并发布到 `sensor_preflight` 主题中。

### 实现

模块在它自己的线程中运行，并且轮训当前选择的 gyro 话题。

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

控制 & 测试（外置）蜂鸣器的命令行工具。

蜂鸣器被用于提供听觉通知和警告（例如，系统解锁、位置锁定等）。 本工具要求一个可处理 tune_control uorb 主题的驱动处于运行状态。

有关音调格式和预定义的系统蜂鸣声音可以参阅： https://github.com/PX4/Firmware/blob/master/src/lib/tunes/tune_definition.desc

### 示例

播放系统蜂鸣声 #2 ：

    tune_control play -t 2
    

### 用法 {#tune_control_usage}

    tune_control <command> [arguments...]
     Commands:
       play          Play system tune or single note.
         [-t <val>]  Play predefined system tune
                     default: 1
         [-f <val>]  Frequency of note in Hz (0-22kHz)
         [-d <val>]  Duration of note in us
         [-s <val>]  Volume level (loudness) of the note (0-100)
                     default: 40
         [-m <val>]  Melody in string form
                     values: <string> - e.g. "MFT200e8a8a"
    
       libtest       Test library
    
       stop          Stop playback (use for repeated tunes)