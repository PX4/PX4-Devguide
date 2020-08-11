# uORB 메시징

## 소개

uORB는 스레드간/프로세스간 통신을 위해 사용되는 비동기 Pub/Sub 메시징 API입니다.

C++에서 어떻게 사용하는지는 [여기](../apps/hello_sky.md)를 봐주세요.

uORB는 많은 어플리케이션이 의존하고 있기 때문에 부트업시에 자동적으로 시작됩니다. `uorb start` 로 시작됩니다. 단위 테스트는 `uorb_tests`를 통해 수행할 수 있습니다.

## 새로운 토픽 추가하기

새로운 uORB 토픽은 메인 PX4 펌웨어 저장소나 독립 브랜치의 메시지 정의에 추가하여 사용할 수 있습니다. 독립적인 브랜치에 uORB 메시지 정의에 추가하는 것은 [이 섹션](../advanced/out_of_tree_modules.md#uorb_message_definitions)을 참고하세요.

새로운 토픽을 만들기 위해서는 `msg/` 디렉토리에 **.msg** 파일을 만들고 `msg/CMakeLists.txt` 리스트에 추가해야합니다. 필요한 C/C++ 코드는 자동적으로 생성됩니다.

지원되는 타입들을 확인하기 위해서는 이미있는 `msg` 파일들을 살펴보세요. 하나의 메시지는 다른 메시지에 포함될 수도 있습니다.

생성된 C/C++ 구조체에는 `uint64_t timestamp` 필드가 추가됩니다. 로깅을 위해 사용되며 메시지를 퍼블리시할때 설정해줘야 합니다.

만든 토픽을 사용하기 위해서는 헤더를 포함해야합니다.

    #include <uORB/topics/topic_name.h>
    

`.msg` 파일에 한줄을 추가함으로써, 하나의 메시지 정의를 다수의 독립된 토픽들을 위해 사용할 수 있습니다.

    # TOPICS mission offboard_mission onboard_mission
    

그리고 소스코드에서 토픽 ID `ORB_ID(offboard_mission)`로 사용하세요.

## 퍼블리시

토픽을 퍼블리싱하는 것은 인터럽트 컨텐스를 포함하는 어느 시스템의 어디에서나 수행할 수 있습니다( `hrt_call` API에 의해 호출됨). 그러나, 토픽을 advertising 하는 것은 인터럽트 컨텍스트의 외부에서만 가능합니다. 토픽을 나중에 퍼블리시할때와 동일한 프로세스에서 Advertise 해야합니다.

## 토픽 리스팅과 리스닝

> **Note** `listener` 명령어는 Pixracer(FMUv4)와 Linux / OS X 에서만 사용가능 합니다.

모든 토픽을 리스팅하기 위해서는 파일 핸들들을 리스팅해야 합니다.

```sh
ls /obj
```

5개의 메시지에 대해 하나의 토픽에 대한 컨텐츠를 수신하고 싶다면,

```sh
listener sensor_accel 5
```

출력은 토픽 내용의 n배 입니다.

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

> **Tip** NuttX기반의 시스템(Pixhawk, Pixracer 등)는 `listener`를 *QGroundControl* MAVLink Console 에서 센서나 다른 토픽들을 검사하기 위해 호출할 수 있습니다. 이 방법은 QGC가 무선으로 연결되어 있을 때 (예. 비행 중 일때)에도 사용할 수 있기 때문에 강력한 디버깅 툴입니다. 더 많은 정보는 [Sensor/Topic Debugging](../debug/sensor_uorb_topic_debugging.md)를 참고하세요.

### uorb top Command

`uorb top` 명령어는 각 토픽들의 퍼블리시 주기를 리얼타임으로 보여줍니다.

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

컬럼들: 토픽 이름, 다중-인스턴스 인덱스, 구독자 수, 퍼블리시 주기(Hz), 초당 잃어버리는 메시지 수 (모든 구독자수를 대상으로), 큐 크기.

## 멀티-인스턴스

uORB는 `orb_advertise_multi`를 통해 동일한 토픽에 대해 독립적인 여러개의 인스턴스를 퍼블리시 하는 메커니즘을 갖고 있습니다. 이 메커니즘은 퍼블리셔에게 인스턴스의 인덱스를 돌려줍니다. 그러면 Sub은 `orb_subscribe_multi`을 사용하여 어떤 인스턴스를 구독할지 선택해야만 합니다(`orb_subscribe`는 첫번째 인스턴스 구독하기). 다수의 인스턴스를 가지는 것은 동일한 타입의 센서를 여러개 가진 시스템에서 유용합니다.

같은 토픽에 대해 `orb_advertise_multi`과 `orb_advertise`가 섞이지 않도록 유의하세요.

API문서는 [src/modules/uORB/uORBManager.hpp](https://github.com/PX4/Firmware/blob/master/src/modules/uORB/uORBManager.hpp)참고하세요.

## Message/Field Deprecation {#deprecation}

As there are external tools using uORB messages from log files, such as [Flight Review](https://github.com/PX4/flight_review), certain aspects need to be considered when updating existing messages:

- Changing existing fields or messages that external tools rely on is generally acceptable if there are good reasons for the update. In particular for breaking changes to *Flight Review*, *Flight Review* must be updated before code is merged to `master`.
- In order for external tools to reliably distinguish between two message versions, the following steps must be followed: 
  - Removed or renamed messages must be added to the `deprecated_msgs` list in [msg/CMakeLists.txt](https://github.com/PX4/Firmware/blob/master/msg/CMakeLists.txt#L157) and the **.msg** file needs to be deleted.
  - Removed or renamed fields must be commented and marked as deprecated. For example `uint8 quat_reset_counter` would become `# DEPRECATED: uint8 quat_reset_counter`. This is to ensure that removed fields (or messages) are not re-added in future.
  - In case of a semantic change (e.g. the unit changes from degrees to radians), the field must be renamed as well and the previous one marked as deprecated as above.