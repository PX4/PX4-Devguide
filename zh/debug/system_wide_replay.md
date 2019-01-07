# 全系统回放

基于 ORB 消息，可以记录和回放系统的任意部分。

回放可用于基于实际数据测试不同参数值的效果，比较不同的估计值等。

## 系统必备组件

首先需要做的是确定应该回放的是一个还是多个模块。 然后, 确定这些模块的所有输入，即订阅 ORB 主题。 对于全系统回放，这包括所有硬件输入：传感器、rc 输入、mavlink 命令和文件系统。

所有已确定的主题都需要以全速率进行记录（请参阅 [logging](../log/logging.md)）。 对于 `ekf2` 默认记录的主题集已经是这种配置。

重要的是，所有重播的主题只包含一个自动生成字段 `timestamp` 绝对时间戳。 如果有是更多的时间戳，那么它们必须相对于主时间戳。 例如, 请参阅 [sensor_combined.msg](https://github.com/PX4/Firmware/blob/master/msg/sensor_combined.msg)。 造成这种情况的原因如下。

## 用法

- 首先，选择要重播的文件，然后生成目标（从固件目录中）： 
        sh
        export replay=<absolute_path_to_log_file.ulg>
        make px4_sitl_default 这将在单独的生成目录 
    
    `build/px4_sitl_default_replay` 中创建输出（以便参数不会干扰正常生成）。 可以为重播选择任何位置 SITL 生成目标，生成系统通过 `replay` 环境变量知道它处于回放模式。
- 在 `build/px4_sitl_default_replay/tmp/rootfs/orb_publisher.rules` 中添加 ORB 发布者规则文件。 此文件定义允许了发布消息的模块。 格式如下：

    restrict_topics: <topic1>, <topic2>, ..., <topicN>
    module: <module>
    ignore_others: <true/false>
    

这意味着已列出的主题只应该被 `&lt;module&gt;` （此为命令名）发布。 从另一个模块发布到这些主题中的任何一个都将被静默忽略。 如果 `true` `ignore_others`, 则将忽略 `&lt;module&gt;` 发布到其他主题的消息。

对于重播，我们只希望 `replay` 模块能够发布以前标识的主题列表。 因此，对于重播 `ekf2`，规则文件如下所示：

    restrict_topics: sensor_combined, vehicle_gps_position, vehicle_land_detected
    module: replay
    ignore_others: true
    

这将使经常发布这些主题消息的模块不需要关闭系统回放 。

- 可选：`build/px4_sitl_default_replay/tmp/rootfs/replay_params.txt` 中的设置参数覆盖。 此文件应包含 ` &lt;param_name&gt; &lt;value&gt; ` 的列表，例如：

    EKF2_GB_NOISE 0.001
    

默认日志文件的所有参数将被使用。 当一个参数在录制过程中发生改变，同样会在回放时候的正确时间发生更改。 在 `replay_params.txt` 的一个参数会覆盖以前的值，同时日志文件它的变化则不会被录入。 - 可选：从 SD 卡复制 `dataman` 任务文件到编译目录。 一个任务的回放只有在必要的时候才进行。 - 启动回放：

```sh
  make px4_sitl_default jmavsim
```

这会自动打开日志文件，应用参数并开始回放。 一旦完成，将自动报告，进程也会退出。 然后新生成的日志文件可以进行分析，它已 `_replayed` 追加到其文件名。

注意，上面的命令也会显示模拟器，但是取决于回放的内容，并不会显示实际数据。 回放时，可以通过 QGC 等查看姿态改变。

- 最后，取消相关环境变量配置，方便正常编译目标再次使用：

```sh
unset replay
```

### 重要提示

- 在重播过程中，将报告日志文件中的所有退出。 这些在回放时都有负面影响，因此回访期间特别小心不要丢包。
- 目前只能在 "实时" 中回放，这意味着速度与录制完成的速度一样快。 这项工作计划今后延长。
- 时间戳为0的消息将被视为无效，并且不会回放。

## EKF2 回放

这是系统范围回放的特殊化处理，用于快速的 ekf2 回放。 它将自动创建 ORB 发布者规则，其工作方式如下：

- Optionally set `SDLOG_MODE` to 1 to start logging from boot
- Record the log
- To replay:

    export replay_mode=ekf2
    export replay=<abs_path_to_log.ulg>
    make px4_sitl none
    

You can stop it after there's an output like:

    INFO  [replay] Replay done (published 9917 msgs, 2.136 s)
    

The parameters can be adjusted as well. They can be extracted from the log with \(install pyulog with `sudo pip install pyulog` first\):

    ulog_params -i $replay -d ' ' | grep -e '^EKF2' > build/px4_sitl_default_replay/tmp/rootfs/replay_params.txt
    

Then edit the parameters in the file as needed and restart the replay process with `make px4_sitl none`. This will create a new log file.

The location of the generated log is printed with a message like this:

    INFO  [logger] Opened log file: rootfs/fs/microsd/log/2017-03-01/13_30_51_replayed.ulg
    

When finished, use `unset replay; unset replay_mode` to exit the replay mode.

## Behind the Scenes

Replay is split into 3 components:

- a replay module
- ORB publisher rules
- time handling

The replay module reads the log and publishes the messages with the same speed as they were recorded. A constant offset is added to the timestamp of each message to match the current system time (this is the reason why all other timestamps need to be relative). The command `replay tryapplyparams` is executed before all other modules are loaded and applies the parameters from the log and user-set parameters. Then as the last command, `replay trystart` will again apply the parameters and start the actual replay. Both commands do nothing if the environment variable `replay` is not set.

The ORB publisher rules allow to select which part of the system is replayed, as described above. They are only compiled for the posix SITL targets.

The **time handling** is still an **open point**, and needs to be implemented.