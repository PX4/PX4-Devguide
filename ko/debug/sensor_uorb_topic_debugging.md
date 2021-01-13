!REDIRECT "https://docs.px4.io/master/ko/debug/sensor_uorb_topic_debugging.html"

# listener 명령을 활용한 센서/토픽 디버깅

uORB는 스레드/프로세스간 통신용 비동기 `publish()` / `subscribe()` 메세지 처리 API입니다. *QGroundControl MAVLink 콘솔*에서 토픽(메세지)에 들어간 센서값을 검사할 때 `listener` 명령을 활용할 수 있습니다.

> **Tip** QGC를 무선 링크(예: 기체 비행 중)로 연결할 때도 활용할 수 있기 때문에 굉장히 강력한 디버깅 도구입니다.

<span></span>

> **Note** `listener` 명령은 [시스템 콘솔](../debug/system_console.md)과 [MAVLink 셸](../debug/mavlink_shell.md)에서도 활용할 수 있습니다.

<span></span>

> **Note** `listener` 명령은 NuttX 기반 시스템(픽스호크, 픽스레이서 등)과 Linux / OS X 에서만 활용할 수 있습니다.

아래 그림에서는 가속 센서 값을 가져오는 *QGroundControl*의 모습을 보여드립니다.

![QGC MAVLink 콘솔](../../assets/gcs/qgc_mavlink_console_listener_command.png)

어떤 토픽을 쓸 수 있는지 결정하는 방법과 `listener` 호출 방법을 더 알아보시려면 [uORB Messaging > Listing Topics and Listening in](../middleware/uorb.md#listing-topics-and-listening-in)을 살펴보십시오.