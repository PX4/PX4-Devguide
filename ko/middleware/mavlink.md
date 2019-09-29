# MAVLink 메시징

[MAVLink](https://mavlink.io/en/)는 드론 에코시스템을 위해 설계된 아주 가벼운 메시징 프로토콜입니다.

PX4는 *QGroundControl*(그리고 다른 GS)과 통신하기 위해 *MAVLink*를 사용합니다. 그리고 비행 컨트롤러 밖에서 드론 컴포넌트에 연결하기 위한 통합 메커니즘니다(예. 회사의 컴퓨터, MAVLink가 활성화된 카메라 등).

이 프로토콜은 데이터 교환을 위해 다수의 [message](https://mavlink.io/en/messages/)와 [microservices](https://mavlink.io/en/services/)를 정의했습니다(다는 아니지만 많은 messages/services가 PX4에 구현되어 있습니다).

이 튜로리얼은 어떻게 여러분이 정의한 메시지를 지원하게 할 수 있는지 설명합니다.

> **Note** 이 튜토리얼은 여러분이 `msg/ca_trajectory.msg` 의 [custom uORB](../middleware/uorb.md) `ca_trajectory` 메시지와 `mavlink/include/mavlink/v2.0/custom_messages/mavlink_msg_ca_trajectory.h` 의 커스텀 MAVLink `ca_trajectory` 메세지를 갖고 있다고 가정합니다.

## 커스텀 MAVLink 메시지 정의하기

MAVLink 개발자 가이드에 새로운 메시지를 어떻게 정의하고 라이브러리에 빌드할지 나와있습니다.

- [How to Define MAVLink Messages & Enums](https://mavlink.io/en/guide/define_xml_element.html)
- [Generating MAVLink Libraries](https://mavlink.io/en/getting_started/generate_libraries.html)

MAVLink2를 위해서는 C 라이브러리로 생성되어야 합니다. 한번 [MAVLink 설치](https://mavlink.io/en/getting_started/installation.html)하기만 하면 다음의 명령어를 통해 수행할 수 있습니다.

```sh
python -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 --output=generated/include/mavlink/v2.0 message_definitions/v1.0/custom_messages.xml
```

사용하기 위해서는 생성된 헤더들을 **Firmware/mavlink/include/mavlink/v2.0**에 복사하기만 하면 됩니다.

변경한 것을 쉽게 테스트하려면, 여러분의 헤더들은 https://github.com/mavlink/c_library_v2를 포크하여 포함하는 것이 나은방법입니다. 그러면 PX4 개발자들은 빌드하기 전에 여러분이 포크한 저장소의 서브모듈을 업데이트할 수 있을 것입니다.

## 커스텀 MAVLink 메시지 보내기

이 섹션에서는 어떻게 uORB 메시지를 사용하기 MAVLink 메시지로 보낼 수 있는지 설명합니다.

MAVLink와 uORB 메시지의 헤더를 [mavlink_messages.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_messages.cpp)에 추가하세요.

```C
#include <uORB/topics/ca_trajectory.h>
#include <v2.0/custom_messages/mavlink.h>
```

[mavlink_messages.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_messages.cpp#L2193)에 새로운 클래스를 생성하세요.

```C
class MavlinkStreamCaTrajectory : public MavlinkStream
{
public:
    const char *get_name() const
    {
        return MavlinkStreamCaTrajectory::get_name_static();
    }
    static const char *get_name_static()
    {
        return "CA_TRAJECTORY";
    }
    static uint16_t get_id_static()
    {
        return MAVLINK_MSG_ID_CA_TRAJECTORY;
    }
    uint16_t get_id()
    {
        return get_id_static();
    }
    static MavlinkStream *new_instance(Mavlink *mavlink)
    {
        return new MavlinkStreamCaTrajectory(mavlink);
    }
    unsigned get_size()
    {
        return MAVLINK_MSG_ID_CA_TRAJECTORY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }

private:
    MavlinkOrbSubscription *_sub;
    uint64_t _ca_traj_time;

    /* do not allow top copying this class */
    MavlinkStreamCaTrajectory(MavlinkStreamCaTrajectory &);
    MavlinkStreamCaTrajectory& operator = (const MavlinkStreamCaTrajectory &);

protected:
    explicit MavlinkStreamCaTrajectory(Mavlink *mavlink) : MavlinkStream(mavlink),
        _sub(_mavlink->add_orb_subscription(ORB_ID(ca_trajectory))),  // make sure you enter the name of your uORB topic here
        _ca_traj_time(0)
    {}

    bool send(const hrt_abstime t)
    {
        struct ca_traj_struct_s _ca_trajectory;    //make sure ca_traj_struct_s is the definition of your uORB topic

        if (_sub->update(&_ca_traj_time, &_ca_trajectory)) {
            mavlink_ca_trajectory_t _msg_ca_trajectory;  //make sure mavlink_ca_trajectory_t is the definition of your custom MAVLink message

            _msg_ca_trajectory.timestamp = _ca_trajectory.timestamp;
            _msg_ca_trajectory.time_start_usec = _ca_trajectory.time_start_usec;
            _msg_ca_trajectory.time_stop_usec  = _ca_trajectory.time_stop_usec;
            _msg_ca_trajectory.coefficients =_ca_trajectory.coefficients;
            _msg_ca_trajectory.seq_id = _ca_trajectory.seq_id;

            mavlink_msg_ca_trajectory_send_struct(_mavlink->get_channel(), &_msg_ca_trajectory)
        }

        return true;
    }
};
```

마지막으로 [mavlink_messages.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_messages.cpp)의 하단에 `streams_list`의 스트림 클래스를 추가하세요.

```C
StreamListItem *streams_list[] = {
...
new StreamListItem(&MavlinkStreamCaTrajectory::new_instance, &MavlinkStreamCaTrajectory::get_name_static, &MavlinkStreamCaTrajectory::get_id_static),
nullptr
};
```

그리고 스트림을 활성화 하세요. 예를 들면, 스타트업 스크립트에 다음의 라인을 추가하는 것입니다. (`-r` 스트리밍 레이트를 설정하고, `-u` MAVLink UDP 포트를 14556로 정합니다):

    mavlink stream -r 50 -s CA_TRAJECTORY -u 14556
    

> **Tip** 여러분의 메시지가 리얼타임으로 동작하는지 확인하기 위해서는 `uorb top [<message_name>]` 명령어를 사용하면됩니다([uORB Messaging](../middleware/uorb.md#uorb-top-command)를 참고하세요). 이 방법은 들어오는 uORB 토픽 메시지를 테스트할 때도 사용할 수 있습니다.(다른 메시지들을 위해서는 코드내에서 `printf` 를 사용하세요).
> 
> *QGroundControl*의 메시지를 보기위해서는 [나만의 MAVLink library](https://dev.qgroundcontrol.com/en/getting_started/)를 빌드하고, 수신하는 메시지를 [MAVLink Inspector Widget](https://docs.qgroundcontrol.com/en/app_menu/mavlink_inspector.html)를 통해 확인할 수 있습니다.

## 커스텀 MAVLink 메시지 수신하기

이 섹션에서는 MAVLink의 메시지를 수신하고 uORB로 퍼블리시 하는 것을 설명합니다.

[mavlink_receiver.h](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.h#L77)에 들어오는 MAVLink 메시지를 핸들링하는 함수를 추가하세요.

```C
#include <uORB/topics/ca_trajectory.h>
#include <v2.0/custom_messages/mavlink_msg_ca_trajectory.h>
```

Add a function that handles the incoming MAVLink message in the [mavlink_receiver.h](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.h#L140) 에 들어오는 MAVLink 메시지를 핸들링하기 위한 함수를`MavlinkReceiver` 클래스안에 추가하세요.

```C
void handle_message_ca_trajectory_msg(mavlink_message_t *msg);
```

[mavlink_receiver.h](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.h#L195)의 `MavlinkReceiver` 클래스에 uORB 퍼블리셔를 추가하세요.

```C
orb_advert_t _ca_traj_msg_pub;
```

[mavlink_receiver.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.cpp)에 `handle_message_ca_trajectory_msg` 함수를 추가하세요.

```C
void MavlinkReceiver::handle_message_ca_trajectory_msg(mavlink_message_t *msg)
{
    mavlink_ca_trajectory_t traj;
    mavlink_msg_ca_trajectory_decode(msg, &traj);

    struct ca_traj_struct_s f;
    memset(&f, 0, sizeof(f));

    f.timestamp = hrt_absolute_time();
    f.seq_id = traj.seq_id;
    f.time_start_usec = traj.time_start_usec;
    f.time_stop_usec = traj.time_stop_usec;
    for(int i=0;i<28;i++)
        f.coefficients[i] = traj.coefficients[i];

    if (_ca_traj_msg_pub == nullptr) {
        _ca_traj_msg_pub = orb_advertise(ORB_ID(ca_trajectory), &f);

    } else {
        orb_publish(ORB_ID(ca_trajectory), _ca_traj_msg_pub, &f);
    }
}
```

and finally make sure it is called in [MavlinkReceiver::handle_message()](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.cpp#L228)

```C
MavlinkReceiver::handle_message(mavlink_message_t *msg)
 {
    switch (msg->msgid) {
        ...
    case MAVLINK_MSG_ID_CA_TRAJECTORY:
        handle_message_ca_trajectory_msg(msg);
        break;
        ...
    }
```

## 커스텀 MAVLink 메시지를 만드는 다른 방법

때로는 아직 완전히 정의되지 않은 내용을 포함하는 MAVLink 메시지를 만들필요가 있습니다.

예를 들어 PX4와 내장된 디바이스의 인터페이스를 MAVLink로 사용할 때 자동조종장치와 그 디바이스는 안전화되기 전에 여러번의 메시지를 교환할 것입니다. 이 작업은 MAVLink 헤더를 재생성하기 위해 시간도 소모되고 에러도 읽어나기 쉽습니다. 그리고 두 장치가 같은 버전의 프로토콜을 사용하는지 확인해야 합니다.

임시적이고 대안적인 방법은 repurpose 디버그 메시지입니다. 커스텀 MAVLink 메시지를 만들기보다는 `CA_TRAJECTORY`, 여러분은 `DEBUG_VECT` 메시지를 문자열 키를 `CA_TRAJ` 에 담고 데이터는 `x`, `y`, `z`에 담아 보낼 수 있습니다. [이 튜토리얼](../debug/debug_values.md)을 참고하세요. 디버그 메시지의 사용예제입니다.

> **Note** 이 방법은 네트워크를 통해 전송하고 문자열 비교를 포함하기 때문에 효율적이지는 않습니다. 개발용으로만 사용하는 것을 권장합니다.

## General

### 스트리밍 레이트 설정하기

때로는 개별적인 토픽들의 스트리밍 레이트를 향상시키는 것이 유용할때가 있습니다(예. QGC 감독). 쉘에 다음과 같은 명령어를 통해 수행할 수 있습니다.

```sh
mavlink stream -u <port number> -s <mavlink topic name> -r <rate>
```

포트넘버와 `transport protocol: UDP (<port number>)`를 출력하는 `mavlink status`를 얻을수도 있습니다. 예:

```sh
mavlink stream -u 14556 -s OPTICAL_FLOW_RAD -r 300
```