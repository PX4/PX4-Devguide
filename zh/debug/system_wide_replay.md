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
    

默认日志文件的所有参数将被使用。 当一个参数在录制过程中发生改变，同样会在回放时候的正确时间发生更改。 A parameter in the `replay_params.txt` will override the value and changes to it from the log file will not be applied. - Optional: copy `dataman` missions file from the SD card to the build directory. Only necessary if a mission should be replayed. - Start the replay:

```sh
  make px4_sitl_default jmavsim
```

This will automatically open the log file, apply the parameters and start to replay. Once done, it will be reported and the process can be exited. Then the newly generated log file can be analyzed, it has `_replayed` appended to its file name.

Note that the above command will show the simulator as well, but depending on what is being replayed, it will not show what's actually going on. It's possible to connect via QGC and e.g. view the changing attitude during replay.

- Finally, unset the environment variable, so that the normal build targets are used again:

```sh
unset replay
```

### Important Notes

- During replay, all dropouts in the log file are reported. These have a negative effect on replay and thus it should be taken care that dropouts are avoided during recording.
- It is currently only possible to replay in 'real-time', meaning as fast as the recording was done. This is planned to be extended in the future.
- A message that has a timestamp of 0 will be considered invalid and not be replayed.

## EKF2 Replay

This is a specialization of the system-wide replay for fast EKF2 replay. It will automatically create the ORB publisher rules and works as following:

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