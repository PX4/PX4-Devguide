# 使用listener命令进行传感器/话题调试

uORB是一个异步`publish()` / `subscribe()` 消息API，用于线程进程间的通信。`listener` 命令可以用于从 *QGroundControl MAVLink Console* 中检查话题（消息）的值，包括传感器当前发布的值。


> **提示：** 这是一个很有效的调试工具，因为它可以在通过无线连接QGC的时候使用（例如：当飞机在飞行的时候）。

<span></span>
> **注意：** `listener`命令也可以在 [System Console](../debug/system_console.md) 以及 [MAVLink Shell](../debug/system_console.md#mavlink-shell) 中使用。

<span></span>
> **注意：** `listener`命令仅在基于NuttX的系统（Pixhawk，Pixracer等）和 Linux/OS X 上可用。

下面的图展示了用*QGroundControl*获得的加速度计的值。

![QGC MAVLink Console](../../assets/gcs/qgc_mavlink_console_listener_command.png)

有关如何确定哪些话题可用以及如何调用`listener`的更多信息，请参阅： [uORB消息机制 > 列出主题并进行监听](../middleware/uorb.md#列出主题并进行监听)。
