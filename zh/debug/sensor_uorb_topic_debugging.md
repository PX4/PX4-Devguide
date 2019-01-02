# 使用侦听器命令进行传感器/主题调试

uORB 是用于进程间通信的异步 `publish()`/`subscribe()` 消息传递 API。 `listener` 命令可从 *QGroundControl 的 MAVLink 终端* 中用于检查主题（消息）值，包括传感器发布的当前值。

> **Tip** 之所以是非常有用的调试工具是因为可以在 QGC 上通过无线连接（比如飞机在飞行过程中）。

<span></span>

> **Note** `listener` 命令也可通过 [System 终端](../debug/system_console.md) 和 [MAVLink shell](../debug/system_console.md#mavlink-shell)。

<span></span>

> **Note** The `listener` command is only available on NuttX-based systems (Pixhawk, Pixracer, etc.) and Linux / OS X.

The image below demonstrates *QGroundControl* being used to get the value of the acceleration sensor.

![QGC MAVLink Console](../../assets/gcs/qgc_mavlink_console_listener_command.png)

For more information about how to determine what topics are available and how to call `listener` see: [uORB Messaging > Listing Topics and Listening in](../middleware/uorb.md#listing-topics-and-listening-in).