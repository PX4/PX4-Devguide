# MAVLink Messaging
전체 메시지에 대한 개요는 다음 링크를 참고하세요. [여기](https://mavlink.io/en/messages/).
## 커스텀 MAVLink Messages 생성
여기 튜터리얼에서는 `msg/ca_trajectory.msg`에 있는 [custom uORB](../middleware/uorb.md) `ca_trajectory`와 `mavlink/include/mavlink/v1.0/custom_messages/mavlink_msg_ca_trajectory.h`에 있는 커스텀 mavlink `ca_trajectory` 메시지를 가지고 있다고 가정합니다. (
[여기](http://qgroundcontrol.org/mavlink/create_new_mavlink_message)에서 커스텀 mavlink 메시지와 헤더 생성하는 방법을 참고).

## 커스텀 MAVLink Message 보내기
이 섹션에서는 커스텀 uORB 메시지를 사용하는 방법과 이를 mavlink 메시지로 보내는 방법을 설명합니다.

mavlink의 헤더와 uorb 메시지를 [mavlink_messages.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_messages.cpp)에 추가합니다.

```C
#include <uORB/topics/ca_trajectory.h>
#include <v1.0/custom_messages/mavlink_msg_ca_trajectory.h>
```

새로운 class를 [mavlink_messages.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_messages.cpp#L2193)에 추가

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
       uint16_t get_id_static()
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
			mavlink_ca_trajectory_t _msg_ca_trajectory;  //make sure mavlink_ca_trajectory_t is the definition of your custom mavlink message

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

마지막으로 stream class를 [mavlink_messages.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_messages.cpp) 끝에 있는 `streams_list`에 추가합니다.

```C
StreamListItem *streams_list[] = {
...
new StreamListItem(&MavlinkStreamCaTrajectory::new_instance, &MavlinkStreamCaTrajectory::get_name_static),
nullptr
};
```

다음으로 stream을 사용하기 위해서 startup 스크립트에 다음 한 줄을 추가할 수 있습니다. (`-r`은 streaming rate를 설정하며 `-u`은 UDP port 14556로 mavlink channel을 식별함) :

```
mavlink stream -r 50 -s CA_TRAJECTORY -u 14556
```


## 커스텀 MAVLink Message 수신하기
이 섹션에서는 mavlink로 메시지를 받고 이를 uORB로 publish하는 방법을 설명합니다.

들어오는 mavlink 메시지를 처리하는 함수를 [mavlink_receiver.h](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.h#L77)에 추가

```C
#include <uORB/topics/ca_trajectory.h>
#include <v1.0/custom_messages/mavlink_msg_ca_trajectory.h>
```

수신하는 mavlink 메시지를 처리하는 함수를 [mavlink_receiver.h](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.h#L140)
에 있는 `MavlinkReceiver` class에 추가

```C
void handle_message_ca_trajectory_msg(mavlink_message_t *msg);
```
uORB publisher를 [mavlink_receiver.h](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.h#L195)에 있는 `MavlinkReceiver` class에 추가

```C
orb_advert_t _ca_traj_msg_pub;
```

`handle_message_ca_trajectory_msg` 함수 구현을 [mavlink_receiver.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.cpp)에 추가

```C
void
MavlinkReceiver::handle_message_ca_trajectory_msg(mavlink_message_t *msg)
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

그리고 마지막으로 [MavlinkReceiver::handle_message()](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.cpp#L228)에서 호출되는지 확인

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

## Alternative to Creating Custom MAVLink Messages
Sometimes there is the need for a custom MAVLink message with content that is not fully defined.

For example when using MAVLink to interface PX4 with an embedded device, the messages that are exchanged between the autopilot and the device may go through several iterations before they are stabilized.
In this case, it can be time-consumming and error-prone to regenerate the MAVLink headers, and make sure both devices use the same version of the protocol.

An alternative - and temporary - solution is to repurpose debug messages.
Instead of creating a custom MAVLink message `CA_TRAJECTORY`, you can send a message `DEBUG_VECT` with the string key `CA_TRAJ` and data in the `x`, `y` and `z` fields.
See [this tutorial](../debug/debug_values.md). for an example usage of debug messages.

> **Note** This solution is not efficient as it sends character string over the network and involves comparison of strings. It should be used for development only!

## 일반
### streaming rate 설정
가끔은 개별 topic의 streaming rate를 올리는 것이 유용합니다. (예로 QGC에서 inspection용도로) 다음과 같은 라인으로 처리할 수 있습니다.
```sh
mavlink stream -u <port number> -s <mavlink topic name> -r <rate>
```
```mavlink status```으로 port number를 가져올 수 있습니다. 출력 중에 ```transport protocol: UDP (<port number>)```를 확인합니다. 예제로
```sh
mavlink stream -u 14556 -s OPTICAL_FLOW_RAD -r 300
```
