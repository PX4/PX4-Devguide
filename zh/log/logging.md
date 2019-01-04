# 日志记录

日志能够记录任何 orb 主题及其包含的所有字段。 所有需要的数据都是从` .msg `文件中产生的，因此只需要指定出题的名称。 可选的间隔参数指定了主题的最大日志记录 速率。 所有主题的实例将会被记录。

输出的日志格式是[ Ulog ](../log/ulog_file_format.md)。

## 用法

默认情况下，日志会在解锁时自动记录，并在加锁时停止。 每次解锁后的飞行对话将会在 SD 卡上生成一个新的日志文件。 要显示当前状态，可以在控制台上输入 `logger status`。 如果你想立即开始日志记录, 请使用 `logger on</0 >。 这将覆盖解锁状态，如果系统已解锁。 <code>log off` 取消日志记录。

使用

    logger help
    

列举所有支持的日志命令和参数。

## 配置

日志主题列表可以以 SD 卡文件的形式定制。 在 SD 卡上创建一个 `etc/logging/logger_topics.txt` 文件，其中包含主题列表（对于SITL，则是`build/px4_sitl_default/tmp/rootfs/fs/microsd/etc/logging/logger_topics.txt`）：

    <topic_name>, <interval>
    

`&lt;interval&gt;`是一个可选项，如果指定，则以 ms 为单位定义两条日志信息的最小记录间隔。 如果未指定,，则全速率记录主题信息。

文件中的主题名将替换所有默认记录的主题。

## 脚本

在 [pyulog](https://github.com/PX4/pyulog) 存储库中有几个脚本来分析和转换日志记录文件。

## 丢帧

日志丢帧是不希望发生的，下面有几个因素对影响丢帧的数量：

- 我们测试的大多数 sd 卡每分钟都会有多个停顿。 这种停顿在写命令期间有好几个 100ms 的延迟。 如果写缓冲区在这期间被填满会引起丢帧。 这种影响取决于 SD 卡本身（见下文）。
- 格式化 SD 卡有助于避免丢帧。
- 增大日志缓存也有效。
- Decrease the logging rate of selected topics or remove unneeded topics from being logged (`info.py <file>` is useful for this).

## SD 卡

下面提供了不同 SD 卡的性能。 测试是在 Pixracer上进行的，这个结果也适用于 Pixhawk。

> **Tip** The maximum supported SD card size for NuttX is 32GB (SD Memory Card Specifications Version 2.0).

| SD 卡                                                          | Mean Seq. 写入速度 [KB/s] | 最大写入时间 / 块（平均） [ms] |
| ------------------------------------------------------------- | --------------------- | ------------------- |
| SanDisk Extreme U3 32GB                                       | 461                   | **15**              |
| Sandisk Ultra Class 10 8GB                                    | 348                   | 40                  |
| Sandisk Class 4 8GB                                           | 212                   | 60                  |
| SanDisk Class 10 32 GB (High Endurance Video Monitoring Card) | 331                   | 220                 |
| Lexar U1 (Class 10), 16GB High-Performance                    | 209                   | 150                 |
| Sandisk Ultra PLUS Class 10 16GB                              | 196                   | 500                 |
| Sandisk Pixtor Class 10 16GB                                  | 334                   | 250                 |
| Sandisk Extreme PLUS Class 10 32GB                            | 332                   | 150                 |

More important than the mean write speed is the maximum write time per block (of 4 KB). This defines the minimum buffer size: the larger this maximum, the larger the log buffer needs to be to avoid dropouts. Logging bandwidth with the default topics is around 50 KB/s, which all of the SD cards satisfy.

By far the best card we know so far is the **SanDisk Extreme U3 32GB**. This card is recommended, because it does not exhibit write time spikes (and thus virtually no dropouts). Different card sizes might work equally well, but the performance is usually different.

You can test your own SD card with `sd_bench -r 50`, and report the results to https://github.com/PX4/Firmware/issues/4634.

## Log Streaming

The traditional and still fully supported way to do logging is using an SD card on the FMU. However there is an alternative, log streaming, which sends the same logging data via MAVLink. This method can be used for example in cases where the FMU does not have an SD card slot (e.g. Intel® Aero Ready to Fly Drone) or simply to avoid having to deal with SD cards. Both methods can be used independently and at the same time.

The requirement is that the link provides at least ~50KB/s, so for example a WiFi link. And only one client can request log streaming at the same time. The connection does not need to be reliable, the protocol is designed to handle drops.

There are different clients that support ulog streaming:

- `mavlink_ulog_streaming.py` script in Firmware/Tools.
- QGroundControl： ![](../../assets/gcs/qgc-log-streaming.png)
- [MAVGCL](https://github.com/ecmnet/MAVGCL)

### 诊断

- If log streaming does not start, make sure the `logger` is running (see above), and inspect the console output while starting.
- If it still does not work, make sure that Mavlink 2 is used. Enforce it by setting `MAV_PROTO_VER` to 2.
- Log streaming uses a maximum of 70% of the configured mavlink rate (`-r` parameter). If more is needed, messages are dropped. The currently used percentage can be inspected with `mavlink status` (1.8% is used in this example):

    instance #0:
            GCS heartbeat:  160955 us ago
            mavlink chan: #0
            type:           GENERIC LINK OR RADIO
            flow control:   OFF
            rates:
            tx: 95.781 kB/s
            txerr: 0.000 kB/s
            rx: 0.021 kB/s
            rate mult: 1.000
            ULog rate: 1.8% of max 70.0%
            accepting commands: YES
            MAVLink version: 2
            transport protocol: UDP (14556)
    

Also make sure `txerr` stays at 0. If this goes up, either the NuttX sending buffer is too small, the physical link is saturated or the hardware is too slow to handle the data.