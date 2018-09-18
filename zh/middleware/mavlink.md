# MAVLink通讯

点击[这里](https://mavlink.io/en/messages/)查看关于所有的系统默认Mavlink消息的概述。

## 创建自定义MAVLink消息

在阅读此教程前，请确保如下事项：（1）`msg/ca_trajectory.msg`中已存在[自定义 uORB消息](../middleware/uorb.md)`ca_trajectory`；（2）头文件`mavlink/include/mavlink/v1.0/custom_messages/mavlink_msg_ca_trajectory.h`中已存在自定义MAVLink消息`ca_trajectory`（点击[这里](http://qgroundcontrol.org/mavlink/create_new_mavlink_message)查看：如何创建自定义MAVLink消息和头文件）。

## 发送自定义MAVLink消息

此章节旨在说明：如何使用一条自定义uORB消息，并将其作为一条MAVLink消息发送出去。

Step1. 首先，在[mavlink_messages.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_messages.cpp)中添加自定义MAVLink消息和uORB消息的头文件：

```C
#include <uORB/topics/ca_trajectory.h>
#include <v1.0/custom_messages/mavlink_msg_ca_trajectory.h>
```

Step2. 然后，在[mavlink_messages.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_messages.cpp#L2193)中新建一个消息流的类：

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
    uint8_t get_id()
    {
        return MAVLINK_MSG_ID_CA_TRAJECTORY;
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

    void send(const hrt_abstime t)
    {
        struct ca_traj_struct_s _ca_trajectory;    //make sure ca_traj_struct_s is the definition of your uORB topic

        if (_sub->update(&_ca_traj_time, &_ca_trajectory)) {
            mavlink_ca_trajectory_t _msg_ca_trajectory;  //make sure mavlink_ca_trajectory_t is the definition of your custom MAVLink message

            _msg_ca_trajectory.timestamp = _ca_trajectory.timestamp;
            _msg_ca_trajectory.time_start_usec = _ca_trajectory.time_start_usec;
            _msg_ca_trajectory.time_stop_usec  = _ca_trajectory.time_stop_usec;
            _msg_ca_trajectory.coefficients =_ca_trajectory.coefficients;
            _msg_ca_trajectory.seq_id = _ca_trajectory.seq_id;

            _mavlink->send_message(MAVLINK_MSG_ID_CA_TRAJECTORY, &_msg_ca_trajectory);
        }
    }
};
```

Step3. 接下来，在[mavlink_messages.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_messages.cpp)的文件末尾，将这个消息流的类追加到`treams_list`。

```C
StreamListItem *streams_list[] = {
...
new StreamListItem(&MavlinkStreamCaTrajectory::new_instance, &MavlinkStreamCaTrajectory::get_name_static),
nullptr
};
```

Step4. 最后，确保使能了此消息流：比如，添加如下内容至启动脚本文件（`-r`+n 命令可设置串流速率，`-u`命令可将MAVLink通道映射到UDP端口14556）

    mavlink stream -r 50 -s CA_TRAJECTORY -u 14556
    

## 接收自定义MAVLink消息

此章节旨在说明：如何接收一条MAVLink消息，并将其发布至uORB。

Step1. 在[mavlink_receiver.h](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.h#L77)中添加自定义MAVLink消息和uORB消息的头文件：

```C
#include <uORB/topics/ca_trajectory.h>
#include <v1.0/custom_messages/mavlink_msg_ca_trajectory.h>
```

Step2. 在[mavlink_receiver.h](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.h#L140)中添加处理自定义MAVLink消息的函数：

```C
void handle_message_ca_trajectory_msg(mavlink_message_t *msg);
```

Step3. 在[mavlink_receiver.h](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.h#L195)中，向类`MavlinkReceiver`中添加uORB发布者类型的成员变量：

```C
orb_advert_t _ca_traj_msg_pub;
```

Step4. 在[mavlink_receiver.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.cpp)中给出函数`handle_message_ca_trajectory_msg`的具体实现：

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

Step5. 最后，确保上述自定义函数在[MavlinkReceiver::handle_message()](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.cpp#L228)中被调用：

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

## 另一种自定义MAVlink消息的办法

Sometimes there is the need for a custom MAVLink message with content that is not fully defined.

For example when using MAVLink to interface PX4 with an embedded device, the messages that are exchanged between the autopilot and the device may go through several iterations before they are stabilized. In this case, it can be time-consuming and error-prone to regenerate the MAVLink headers, and make sure both devices use the same version of the protocol.

An alternative - and temporary - solution is to re-purpose debug messages. Instead of creating a custom MAVLink message `CA_TRAJECTORY`, you can send a message `DEBUG_VECT` with the string key `CA_TRAJ` and data in the `x`, `y` and `z` fields. See [this tutorial](../debug/debug_values.md). for an example usage of debug messages.

> **Note** This solution is not efficient as it sends character string over the network and involves comparison of strings. It should be used for development only!

## General

### Set streaming rate

Sometimes it is useful to increase the streaming rate of individual topics (e.g. for inspection in QGC). This can be achieved by typing the following line in the shell:

```sh
mavlink stream -u <port number> -s <mavlink topic name> -r <rate>
```

You can get the port number with `mavlink status` which will output (amongst others) `transport protocol: UDP (<port number>)`. An example would be

```sh
mavlink stream -u 14556 -s OPTICAL_FLOW_RAD -r 300
```