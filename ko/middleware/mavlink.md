# MAVLink 메세징

[MAVLink](https://mavlink.io/en/)는 드론 생태계에서 활용하려 설계한 초경량 메세지 프로토콜입니다.

PX4는 *QGroundControl*(그리고 기타 지상 통제 장치)과의 통신, 그리고 보조 컴퓨터, MAVLink 기능을 갖춘 카메라 등 비행체 제어 장치 외의 드론 구성요소 연결을 위한 통합 매커니즘 용도로 *MAVLink* 를 활용합니다.

프로토콜은 데이터 교환 목적의 표준 [메세지](https://mavlink.io/en/messages/)와 [마이크로서비스](https://mavlink.io/en/services/)를 정의합니다(여러가지가 있으나, PX4 용도로의 메시지/서비스를 전부 구현하진 못했습니다).

이 자습서에서는 새 "개별 정의" 메시지를 PX4 지원에 추가하는 방법을 알려드리겠습니다.

> **Note** 이 자습서에서는 이미 `msg/ca_trajectory.msg`에 [custom uORB](../middleware/uorb.md) `ca_trajectory` 메세지를 정의하고, 개별 MAVLink `ca_trajectory`메세지를 `mavlink/include/mavlink/v2.0/custom_messages/mavlink_msg_ca_trajectory.h` 에 정의했음을 가정합니다.

## MAVLink 개별 메세지 정의

MAVLink 개발자 안내서에서는 프로그래밍 영역의 라이브러리에 새 메세지 정의하고 빌드하는 방법을 설명합니다:

- [How to Define MAVLink Messages & Enums](https://mavlink.io/en/guide/define_xml_element.html)
- [MAVLink 라이브러리 생성](https://mavlink.io/en/getting_started/generate_libraries.html)

MAVLink 2 메세지는 C 라이브러리로 만들어야합니다. 일단 [MAVLink를 설치](https://mavlink.io/en/getting_started/installation.html)하면, 아래 명령으로 명령행에서 메세지 정의 및 빌드를 처리할 수 있습니다:

```sh
python -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 --output=generated/include/mavlink/v2.0 message_definitions/v1.0/custom_messages.xml
```

새로 만든 헤더를 활용/시험 목적으로 **Firmware/mavlink/include/mavlink/v2.0**에 복사할 수 있습니다.

기존의 다른 요소와 바뀐 내용을 쉽게 시험해보려면, https://github.com/mavlink/c_library_v2 저장소를 가져온 소스트리에 새로 만든 헤더를 추가하는 방법이 가장 바람직한 접근 방안이라 볼 수 있습니다. PX4 개발자의 경우 가져온 하위 모듈을 빌드를 진행하기 전에 펌웨어 저장소에서 업데이트할 수 있습니다.

## MAVLink 개별 메시지 송신

이 절에서는 uORB 개별 설정 메세지를 활용하여 MAVLink 메세지에 실어 보내는 방법을 설명합니다.

[mavlink_messages.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_messages.cpp)에 MAVLink 헤더와 uORB 메세지를 추가하십시오.

```C
#include <uORB/topics/ca_trajectory.h>
#include <v2.0/custom_messages/mavlink.h>
```

[mavlink_messages.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_messages.cpp#L2193)에 새 클래스를 만드십시오.

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

마지막으로 [mavlink_messages.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_messages.cpp)의 하단에 `streams_list`의 스트림 클래스를 추가하십시오.

```C
StreamListItem *streams_list[] = {
...
new StreamListItem(&MavlinkStreamCaTrajectory::new_instance, &MavlinkStreamCaTrajectory::get_name_static, &MavlinkStreamCaTrajectory::get_id_static),
nullptr
};
```

그 다음 [시작 스크립트](../concept/system_startup.md) (예: NuttX의 [/ROMFS/px4fmu_common/init.d-posix/rcS](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d-posix/rcS) 또는 SITL의 [ROMFS/px4fmu_common/init.d-posix/rcS](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d-posix/rcS)) 에 다음 행을 추가하는 식으로 스트림을 활성화했는지 확인하십시오. 참고로, `-r` 인자로 스트리밍 전송율을 설정하고 `-u` 인자로 UDP 포트 14556의 MAVLink 채널을 식별합니다.

    mavlink stream -r 50 -s CA_TRAJECTORY -u 14556
    

> **Tip** `uorb top [<message_name>]` 명령을 활용하여 메시지 송신 여부와 전송율을 실시간으로 확인할 수 있습니다([uORB 메세징](../middleware/uorb.md#uorb-top-command) 참고). 이 방법으로 uORB 토픽을 내보내는 메시지 수신을 시험해볼 수 있습니다(다른 메시지에 대해서는 SITL에서 코드와 테스트에 `printf`를 사용해야 합니다).
> 
> *QGroundControl*에서 메세지를 보려면 [MAVLink 라이브러리를 빌드](https://dev.qgroundcontrol.com/en/getting_started/)하고 [MAVLink 검사 위젯](https://docs.qgroundcontrol.com/en/app_menu/mavlink_inspector.html)(또는 다른 MAVLink 도구)으로 수신 메시지를 확인해야 합니다.

## MAVLink 개별 메시지 수신

이 절에서는 MAVLink 메시지 수신 및 uORB 대상 송신 방법을 설명합니다.

[mavlink_receiver.h](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.h#L77)에 MAVLink 메세지 수신 핸들 함수를 추가하십시오.

```C
#include <uORB/topics/ca_trajectory.h>
#include <v2.0/custom_messages/mavlink_msg_ca_trajectory.h>
```

[mavlink_receiver.h](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.h#L140)의 `MavlinkReceiver` 클래스에 MAVLink 메세지 수신을 처리할 핸들 함수를 추가하십시오.

```C
void handle_message_ca_trajectory_msg(mavlink_message_t *msg);
```

[mavlink_receiver.h](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.h#L195)의 `MavlinkReceiver` 클래스에 uORB 송신부를 추가하십시오.

```C
orb_advert_t _ca_traj_msg_pub;
```

[mavlink_receiver.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.cpp)에 `handle_message_ca_trajectory_msg` 함수 구현체를 넣으십시오.

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

## MAVLink 개별 메세지 생성 대안

때로는 완전히 정의하지 못한 내용이 담긴 MAVLink 개별 메세지가 필요할 경우가 있습니다.

예를 들어 PX4와 내장된 디바이스의 인터페이스를 MAVLink로 사용할 때 자동조종장치와 그 디바이스는 안전화되기 전에 여러번의 메시지를 교환할 것입니다. 이 과정은 오류에 취약한 MAVLink 헤더를 다시 만들고 장치간 프로토콜 버전의 동일 여부를 확인하는데 시간을 소요할 수 있습니다.

임시 대안책은 디버깅 메시지의 목적 전환입니다. MAVLink 개별 메세지 `CA_TRAJECTORY`를 만드는 대신, `CA_TRAJ` 문자열 키와 `x`, `y`, `z` 필드에 데이터를 담은 `DEBUG_VECT` 메세지를 보낼 수 있습니다. [이 자습서](../debug/debug_values.md)를 살펴보십시오. 디버깅 메시지 사용 예제가 들어있습니다.

> **Note** 이 방법은 문자열을 네트워크로 보내면서 문자열 비교과정에 관여하므로 그다지 효율적이지 않습니다. 개발용으로만 활용하십시오!

## 일반

### 스트리밍 전송율 설정

개별 토픽의 스트리밍 전송율을 늘리는 방법이 좋을 때가 있습니다(QGC 에서의 상태 검사). 셸에 다음 명령을 입력해서 처리할 수 있습니다:

```sh
mavlink stream -u <port number> -s <mavlink topic name> -r <rate>
```

(다른 내용과 함께) `transport protocol: UDP (<port number>)`를 출력하는 `mavlink status` 명령으로 포트 번호를 알 수 있습니다. 예제는 다음과 같습니다:

```sh
mavlink stream -u 14556 -s OPTICAL_FLOW_RAD -r 300
```