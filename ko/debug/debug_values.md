# 디버깅 값 송수신

소프트웨어 개발 과정에서 제각각 중요한 숫자 값을 출력할 필요가 종종 있습니다. 이 때가 보통 MAVLink 패킷이 들어올 때 `NAMED_VALUE_FLOAT`, `DEBUG`, `DEBUG_VECT` 패킷을 활용할 수 있는 경우입니다.

## MAVLink 디버깅 메시지와 uORB 토픽 대응

MAVLink 디버깅 메시지는 uORB 토픽으로 변환하거나 그 반대로 재변환할 수 있습니다. MAVLink 디버깅 메시지를 송수신하려면, 각각의 해당 토픽을 보내(publish)거나 지속적으로 수신(subscribe)해야합니다. 아래 표를 통해 MAVLink 디버깅 메시지와 uORB 토픽의 대응을 정리해드렸습니다:

| MAVLink 메세지         | uORB 토픽           |
| ------------------- | ----------------- |
| NAMED_VALUE_FLOAT | debug_key_value |
| DEBUG               | debug_value       |
| DEBUG_VECT          | debug_vect        |

## 자습서: 문자열 / 부동소숫점 값 보내기

이번 따라하기 절에서는 MAVLink 메세지 `NAMED_VALUE_FLOAT`를 uORB 토픽의 `debug_key_value`로 보내는 방법을 알려드리도록 하겠습니다.

이 자습서에 있는 코드는 다음과 같습니다:

* [디버깅 자습 코드](https://github.com/PX4/Firmware/blob/master/src/examples/px4_mavlink_debug/px4_mavlink_debug.cpp)
* MAVLink 디버깅 앱(**px4_mavlink_debug**)에서 보드 설정의 주석을 해제하여 하나하나 확인하는 과정을 통해 [자습서 앱 동작을 활성화](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v5/default.cmake)합니다.

디버깅 내용 출력을 설정하는데 필요한 모든 구성은 이 코드 조각에 다 들어있습니다. 우선 헤더 파일을 추가해보도록 하겠습니다:

```C
#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <string.h>
```

그 다음 디버깅 값 토픽(다른 공개 이름에 대해 한번의 광역 송신으로 충분합니다)을 여러 노드로 보내겠습니다. 이 코드를 메인 루프 앞에 붙여넣으십시오.

```C
/* advertise debug value */
struct debug_key_value_s dbg;
strncpy(dbg.key, "velx", sizeof(dbg.key));
dbg.value = 0.0f;
orb_advert_t pub_dbg = orb_advertise(ORB_ID(debug_key_value), &dbg);
```

이렇게 하면 메인 루프에서 메시지 전송은 상당히 간단합니다:

```C
dbg.value = position[0];
orb_publish(ORB_ID(debug_key_value), pub_dbg, &dbg);
```

> **Caution** 다중 디버깅 메시지를 보내서 MAVLink로 처리하는데 충분한 시간을 주어야합니다. 이는 여러 디버깅 메시지를 내보니거나 각 함수 호출 반복간 메시지 전송시 대기 시간을 주어야 한다는 의미입니다.

QGroundControl에서는 아래와 같은 실시간 플롯(2차원 도표)을 보여줍니다:

![QGC debugvalue plot](../../assets/gcs/qgc-debugval-plot.jpg)

## 자습서: 문자열 / 부동소숫점 값 받기

The following code snippets show how to receive the `velx` debug variable that was sent in the previous tutorial.

First, subscribe to the topic `debug_key_value`:

```C
#include <poll.h>
#include <uORB/topics/debug_key_value.h>

int debug_sub_fd = orb_subscribe(ORB_ID(debug_key_value));
[...]
```

이후 토픽을 폴링 처리하십시오:

```C
[...]
/* one could wait for multiple topics with this technique, just using one here */
px4_pollfd_struct_t fds[] = {
    { .fd = debug_sub_fd,   .events = POLLIN },
};

while (true) {
    /* wait for debug_key_value for 1000 ms (1 second) */
    int poll_ret = px4_poll(fds, 1, 1000);

    [...]
```

When a new message is available on the `debug_key_value` topic, do not forget to filter it based on its key attribute in order to discard the messages with key different than `velx`:

```C
    [...]
    if (fds[0].revents & POLLIN) {
        /* obtained data for the first file descriptor */
        struct debug_key_value_s dbg;

        /* copy data into local buffer */
        orb_copy(ORB_ID(debug_key_value), debug_sub_fd, &dbg);

        /* filter message based on its key attribute */
        if (strcmp(_sub_debug_vect.get().key, "velx") == 0) {
            PX4_INFO("velx:\t%8.4f", dbg.value);
        }
    }
}

```