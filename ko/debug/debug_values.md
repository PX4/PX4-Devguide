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

The code for this tutorial is available here:

* [Debug Tutorial Code](https://github.com/PX4/Firmware/blob/master/src/examples/px4_mavlink_debug/px4_mavlink_debug.cpp)
* [Enable the tutorial app](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v5/default.cmake) by ensuring the MAVLink debug app (**px4_mavlink_debug**) is uncommented in the config of your board.

All required to set up a debug publication is this code snippet. First add the header file:

```C
#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <string.h>
```

Then advertise the debug value topic (one advertisement for different published names is sufficient). Put this in front of your main loop:

```C
/* advertise debug value */
struct debug_key_value_s dbg;
strncpy(dbg.key, "velx", sizeof(dbg.key));
dbg.value = 0.0f;
orb_advert_t pub_dbg = orb_advertise(ORB_ID(debug_key_value), &dbg);
```

And sending in the main loop is even simpler:

```C
dbg.value = position[0];
orb_publish(ORB_ID(debug_key_value), pub_dbg, &dbg);
```

> **Caution** Multiple debug messages must have enough time between their respective publishings for Mavlink to process them. This means that either the code must wait between publishing multiple debug messages, or alternate the messages on each function call iteration.

The result in QGroundControl then looks like this on the real-time plot:

![QGC debugvalue plot](../../assets/gcs/qgc-debugval-plot.jpg)

## Tutorial: Receive String / Float Pairs

The following code snippets show how to receive the `velx` debug variable that was sent in the previous tutorial.

First, subscribe to the topic `debug_key_value`:

```C
#include <poll.h>
#include <uORB/topics/debug_key_value.h>

int debug_sub_fd = orb_subscribe(ORB_ID(debug_key_value));
[...]
```

Then poll on the topic:

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