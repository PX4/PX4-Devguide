---
translated_page: https://github.com/PX4/Devguide/blob/master/en/middleware/mavlink.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# MAVLink消息

所有消息的概述可以在[这里](https://mavlink.io/en/messages/)找到.

## 创建自定义MAVLink消息

这篇教程是假设你已经在 `msg/ca_trajectory.msg` 有了一个[自定义uORB](../middleware/uorb.md) `ca_trajectory`
消息，并且在 `mavlink/include/mavlink/v1.0/custom_messages/mavlink_msg_ca_trajectory.h` 有了一个自定义mavlink
`ca_trajectory` 消息（点击[此处](http://qgroundcontrol.org/mavlink/create_new_mavlink_message)查看如何建立一个自定义mavlink消息以及头文件）。



## 发送自定义MAVLink消息


这部分介绍如何使用一个自定义uORB消息并且作为mavlink消息发送。
添加`mavlink`的头文件和uorb消息到[mavlink_messages.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_messages.cpp)
```C
#include <uORB/topics/ca_trajectory.h>
#include <v1.0/custom_messages/mavlink_msg_ca_trajectory.h>
```

在[mavlink_messages.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_messages.cpp#L2193)中创建一个新的类
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

    static uint8_t get_id_static()
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
		_sub(_mavlink->add_orb_subscription(ORB_ID(ca_trajectory))),  // make sure you enter the name of your uorb topic here
		_ca_traj_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		struct ca_trajectory_s _ca_trajectory;    //make sure ca_trajectory_s is the definition of your uorb topic

		if (_sub->update(&_ca_traj_time, &_ca_trajectory)) {

			mavlink_ca_trajectory_t msg;//make sure mavlink_ca_trajectory_t is the definition of your custom mavlink message 

           	msg.timestamp = _ca_trajectory.timestamp;
            msg.time_start_usec = _ca_trajectory.time_start_usec;
            msg.time_stop_usec  = _ca_trajectory.time_stop_usec;
		    msg.coefficients =_ca_trajectory.coefficients;
            msg.seq_id = _ca_trajectory.seq_id;

            mavlink_msg_ca_trajectory_send_struct(_mavlink->get_channel(), &msg);
		}

	return true;
	}
};
```

最后附加流类`streams_list`的到[mavlink_messages.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_messages.cpp)底部
```C
StreamListItem *streams_list[] = {
...
new StreamListItem(&MavlinkStreamCaTrajectory::new_instance, &MavlinkStreamCaTrajectory::get_name_static),
nullptr
};
```

然后确保启用流，例如通过在启动脚本中添加以下行（`-r`配置流速率，`-u`标识UDP端口14556上的mavlink通道）：

```
mavlink stream -r 50 -s CA_TRAJECTORY -u 14556
```

## 接收自定义MAVLink消息

这部分解释如何通过mavlink接收消息并将其发布到uORB。


在[mavlink_receiver.h](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.h#L77)中增加一个用来处理接收信息得函数

```C
#include <uORB/topics/ca_trajectory.h>
#include <v1.0/custom_messages/mavlink_msg_ca_trajectory.h>
```


在 [mavlink_receiver.h](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.h#L140)中增加一个处理类`MavlinkReceiver` 中的输入mavlink消息的函数


```C
void handle_message_ca_trajectory_msg(mavlink_message_t *msg);
```

在 [mavlink_receiver.h](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.h#L195)中加入一个类`MavlinkReceiver`中的uORB消息发布者


```C
orb_advert_t _ca_traj_msg_pub;
```

在[mavlink_receiver.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.cpp)中实现`handle_message_ca_trajectory_msg`功能
```C
void
MavlinkReceiver::handle_message_ca_trajectory_msg(mavlink_message_t *msg)
{
	mavlink_ca_trajectory_t traj;
	mavlink_msg_ca_trajectory_decode(msg, &traj);

	struct ca_trajectory_s f;
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

最后确定函数在[MavlinkReceiver::handle_message()](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.cpp#L228)中被调用

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

## 一般情况

### 设置流速率

有时，增加单个主题的流速率（例如，为例在QGC中检查）是有用的。这可以通过下面这行代码来实现
```sh
mavlink stream -u <port number> -s <mavlink topic name> -r <rate>
```

你可以通过`mavlink status`找到端口号，相应地将输出（在其他消息之间）`transport protocol: UDP (<port number>)`。例如你可能得到
```sh
mavlink stream -u 14556 -s OPTICAL_FLOW_RAD -r 300
```
