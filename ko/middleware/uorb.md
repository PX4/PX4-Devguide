# uORB 메시징

## 소개

uORB는 스레드간/프로세스간 통신을 위해 사용되는 비동기 Pub/Sub 메시징 API입니다.

C++에서 어떻게 사용하는지는 [여기](../apps/hello_sky.md)를 봐주세요.

uORB는 많은 어플리케이션이 의존하고 있기 때문에 부트업시에 자동적으로 시작됩니다. `uorb start` 로 시작됩니다. 단위 테스트는 `uorb_tests`를 통해 수행할 수 있습니다.

## 새로운 토픽 추가하기

새로운 uORB 토픽은 메인 PX4 펌웨어 저장소나 독립 브랜치의 메시지 정의에 추가하여 사용할 수 있습니다. 독립적인 브랜치에 uORB 메시지 정의에 추가하는 것은 [이 섹션](../advanced/out_of_tree_modules.md#uorb_message_definitions)을 참고하세요.

새로운 토픽을 만들기 위해서는 `msg/` 디렉토리에 **.msg** 파일을 만들고 `msg/CMakeLists.txt` 리스트에 추가해야합니다. From this, the needed C/C++ code is automatically generated.

Have a look at the existing `msg` files for supported types. A message can also be used nested in other messages.

To each generated C/C++ struct, a field `uint64_t timestamp` will be added. This is used for the logger, so make sure to fill it in when publishing the message.

To use the topic in the code, include the header:

    #include <uORB/topics/topic_name.h>
    

By adding a line like the following in the `.msg` file, a single message definition can be used for multiple independent topics:

    # TOPICS mission offboard_mission onboard_mission
    

Then in the code, use them as topic id: `ORB_ID(offboard_mission)`.

## Publishing

Publishing a topic can be done from anywhere in the system, including interrupt context (functions called by the `hrt_call` API). However, advertising a topic is only possible outside of interrupt context. A topic has to be advertised in the same process as it's later published.

## Listing Topics and Listening in

> **Note** The `listener` command is only available on Pixracer (FMUv4) and Linux / OS X.

To list all topics, list the file handles:

```sh
ls /obj
```

To listen to the content of one topic for 5 messages, run the listener:

```sh
listener sensor_accel 5
```

The output is n-times the content of the topic:

```sh
TOPIC: sensor_accel #3
timestamp: 84978861
integral_dt: 4044
error_count: 0
x: -1
y: 2
z: 100
x_integral: -0
y_integral: 0
z_integral: 0
temperature: 46
range_m_s2: 78
scaling: 0

TOPIC: sensor_accel #4
timestamp: 85010833
integral_dt: 3980
error_count: 0
x: -1
y: 2
z: 100
x_integral: -0
y_integral: 0
z_integral: 0
temperature: 46
range_m_s2: 78
scaling: 0
```

> **Tip** On NuttX-based systems (Pixhawk, Pixracer, etc) the `listener` command can be called from within the *QGroundControl* MAVLink Console to inspect the values of sensors and other topics. This is a powerful debugging tool because it can be used even when QGC is connected over a wireless link (e.g. when the vehicle is flying). For more information see: [Sensor/Topic Debugging](../debug/sensor_uorb_topic_debugging.md).

### uorb top Command

The command `uorb top` shows the publishing frequency of each topic in real-time:

```sh
update: 1s, num topics: 77
TOPIC NAME                        INST #SUB #MSG #LOST #QSIZE
actuator_armed                       0    6    4     0 1
actuator_controls_0                  0    7  242  1044 1
battery_status                       0    6  500  2694 1
commander_state                      0    1   98    89 1
control_state                        0    4  242   433 1
ekf2_innovations                     0    1  242   223 1
ekf2_timestamps                      0    1  242    23 1
estimator_status                     0    3  242   488 1
mc_att_ctrl_status                   0    0  242     0 1
sensor_accel                         0    1  242     0 1
sensor_accel                         1    1  249    43 1
sensor_baro                          0    1   42     0 1
sensor_combined                      0    6  242   636 1
```

The columns are: topic name, multi-instance index, number of subscribers, publishing frequency in Hz, number of lost messages per second (for all subscribers combined), and queue size.

## Multi-instance

uORB provides a mechanism to publish multiple independent instances of the same topic through `orb_advertise_multi`. It will return an instance index to the publisher. A subscriber will then have to choose to which instance to subscribe to using `orb_subscribe_multi` (`orb_subscribe` subscribes to the first instance). Having multiple instances is useful for example if the system has several sensors of the same type.

Make sure not to mix `orb_advertise_multi` and `orb_advertise` for the same topic.

The full API is documented in [src/modules/uORB/uORBManager.hpp](https://github.com/PX4/Firmware/blob/master/src/modules/uORB/uORBManager.hpp).