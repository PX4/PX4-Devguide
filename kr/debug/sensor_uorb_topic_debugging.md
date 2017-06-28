# Listener 명령을 통해 Sensor/Topic 디버깅

uORB는 비동기 `publish()` / `subscribe()` 메시징 API를 사용해서 thread나 process간 통신이 가능합니다. `listener` 명령은 topic(message)값을 조사하기 위해서 *QGroundControl MAVLink Console* 에서 사용할 수 있습니다. 센서가 publish하는 현재 값도 여기에 포함됩니다.

> **Tip** QGC가 무선으로 연결되어 있더라도 사용할 수 있기 때문에 강력한 디버깅 도구입니다.(예로 비행체가 날고 있는 동안)

<span></span>
> **Note** `listener` 명령은 [System Console](../debug/system_console.md)와 [MAVLink Shell](../debug/system_console.md#mavlink-shell)에서 사용 가능합니다.

<span></span>
> **Note** `listener` 명령은 NuttX 기반 시스템(Pixhawk, Pixracer 등)과 Linux / OS X에서만 사용가능합니다.

아래 이미지는 가속도 센서의 값을 얻는데 사용하는 *QGroundControl* 을 보여줍니다.

![QGC MAVLink Console](../../assets/gcs/qgc_mavlink_console_listener_command.png)

어떤 topic이 유효한지 결정하는 방법과 `listener`를 호출하는 방법에 관한 추가 정보는  [uORB Messaging > Listing Topics and Listening in](../middleware/uorb.md#listing-topics-and-listening-in)을 참고하세요.
