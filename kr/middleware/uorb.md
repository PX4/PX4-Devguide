# uORB Messaging

## 소개

uORB는 thread간/프로세스간 통신을 위해서 비동기 `publish()` / `subscribe()`을 사용합니다.

C++로 사용 방법을 익히려면 [튜토리얼](../tutorials/tutorial_hello_sky.md)을 참고하세요.

uORB는 많은 어플리케이션이 의존하므로 자동으로 부팅시에 일찍이 구동됩니다. `uorb start`로 구동시킵니다. 단위테스트는 `uorb_tests`로 구동시킬 수 있습니다.

## 새로운 topic 추가하기

새로운 topic을 추가하기 위해서 `msg/` 디렉토리에 새로운 `.msg` 파일을 생성하고 파일 이름을 `msg/CMakeLists.txt` 목록에 추가합니다. 이렇게 하면 필요한 C/C++ 코드가 자동으로 생성됩니다.

지원하는 타입을 참고하려면 기존 `msg` 파일을 살펴보세요. 메시지는 다른 메시지의 nest로 사용할 수 있습니다.

각 생성된 C/C++ 구조체에는 `uint64_t timestamp` 필드가 추가되어 있을 것입니다. 이는 logger를 위해 사용되므로 메시지를 publish할 때 값을 넣도록 합니다.

topic을 코드에서 사용하려면 헤더를 포함합니다 :

```
#include <uORB/topics/topic_name.h>
```

`.msg` 파일에 다음과 같은 라인을 추가하면 단일 메시지 정의가 여러 독립적인 topic 인스턴스에서 사용할 수 있습니다. :

```
# TOPICS mission offboard_mission onboard_mission
```

다음으로 코드에서 이를 topic id로 사용 : `ORB_ID(offboard_mission)`.


## Publishing

topic을 publish하는 것은 시스템 어디서든 가능합니다. 인터럽트 컨텍스트(`hrt call` API에서 호출하는 함수)에서도 가능합니다. 그러나 topic을 advertising하는 것은 인터럽트 컨텍스트의 외부에서만 가능합니다. topic은 추후 publish되므로 동일한 프로세스에서 advertise해야만 합니다.

## Topics 목록 및 Listening in

> **Note** `listener` 명령은 Pixracer (FMUv4)와 Linux / OS X에서만 가능합니다.

모든 topic의 목록을 보기 위해 파일 핸들 목록 보기:

```sh
ls /obj
```

5개 메시지에 대해서 한 개 topic의 컨텐트를 liste하기 위해서 listener 실행 :

```sh
listener sensor_accel 5
```

출력은 n 번 topic 콘텐츠입니다. :

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

> **Tip** NuttX-기반 시스템 (Pixhawk, Pixracer 등)에서 `listener` 명령은 *QGroundControl* MAVLink 콘솔내에서 센서의 값이나 다른 topic을 조사하기 위해서 호출할 수 있습니다. QGC가 무선으로 연결되어 있는 경우에도(비행체가 비행 중인 경우) 사용할 수 있으므로 강력한 디버깅 도구가 됩니다. 추가 정보는 [Sensor/Topic Debugging](../debug/sensor_uorb_topic_debugging.md)을 참고하세요.


### urb top 명령

`uorb top` 명령은 실시간으로 각 topic의 publish 주기를 보여줍니다.:

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
컬럼은 : topic 이름, 다중-인스턴스 인덱스, subscriber의 수, Hz 단위 publishing 주기, 손실 메시지 수(모두 합친 subscriber), queue 크기


## 다중-인스턴스

uORB는 `orb_advertise_multi`를 통해 동일한 topic의 다중 독립 인스턴스를 publish하는 매커니즘을 제공합니다. publisher에게 인스턴스 인덱스를 반환합니다. subscriber는 `orb_subscribe_multi`를(`orb_subscribe`는 첫번째 인스턴스를 subscribe) 이용해서 어떤 인스턴스를 선택해야만 합니다.
다중 인스턴스를 가지면 시스템이 동일한 타입의 여러 센서를 가지고 있는 경우 유용합니다.

동일 topic에 대해서 `orb_advertise_multi`와 `orb_advertise`를 섞지 않도록 합니다.

전체 API는 [src/modules/uORB/uORBManager.hpp](https://github.com/PX4/Firmware/blob/master/src/modules/uORB/uORBManager.hpp)를 참고하세요.

## 문제해결 및 일반적인 실수

다음에서는 일반적인 실수나 비정상적인 상황에 대해서 설명합니다 :
- topic이 publish되지 않아요 : 각 호출에서 `ORB_ID()`가 맞는지 확인. `orb_subscribe` 와 `orb_unsubscribe`가 `orb_publish`로서 **동일한 태스크에서 호출** 되었는지가 중요합니다. 이것은 `px4_task_spawn_cmd()`에 적용되지만 work queue(`work_queue()`)를 사용하는 경우에도 적용됩니다.
- 정리(clean up) 하기 : `orb_unsubscribe`와 `orb_unadvertise`를 사용하세요.
- 성공적으로 `orb_check()`나 `px4_poll` 호출은 `orb_copy()`가 필요하고 다음 poll에서 즉시 반환됩니다.
- topic을 advertise하기 전에 `orb_subscribe`를 호출해도 아무 문제 없습니다.
- `orb_check()`와 `px4_poll()`은 `orb_subscribe()` 후에 완료되는 publication에 대해서만 true를 반환합니다. topic이 일정하게 publish되지 않는다는 점이 중요합니다. 만약 subscriber가 이전 데이터를 원하는 경우 `orb_subscribe()`이후에 바로 조건없이 `orb_copy()`를 하면 됩니다.(advertiser가 없는 경우라면 `orb_copy()`는 실패하게 됩니다.)
