# listener 명령을 활용한 센서/토픽 디버깅

uORB는 스레드/프로세스간 통신용 비동기 `publish()` / `subscribe()` 메세지 처리 API입니다. The `listener` command can be used from the *QGroundControl MAVLink Console* to inspect topic (message) values, including the current values published by sensors.

> **Tip** This is a powerful debugging tool because it can be used even when QGC is connected over a wireless link (e.g. when the vehicle is flying).

<span></span>

> **Note** The `listener` command is also available through the [System Console](../debug/system_console.md) and the [MAVLink Shell](../debug/mavlink_shell.md).

<span></span>

> **Note** The `listener` command is only available on NuttX-based systems (Pixhawk, Pixracer, etc.) and Linux / OS X.

The image below demonstrates *QGroundControl* being used to get the value of the acceleration sensor.

![QGC MAVLink Console](../../assets/gcs/qgc_mavlink_console_listener_command.png)

For more information about how to determine what topics are available and how to call `listener` see: [uORB Messaging > Listing Topics and Listening in](../middleware/uorb.md#listing-topics-and-listening-in).