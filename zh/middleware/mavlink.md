# MAVLink 通讯

[MAVLink](https://mavlink.io/en/) 是一个针对无人机生态系统设计的非常轻量化的消息传递协议。

PX4 使用 *MAVLink* 实现与 *QGroundControl* （或者其它地面站软件）的通讯交流，同时也将其用于整合飞控板与飞控板之外的无人机部件：伴随计算机、支持 MAVLink 的摄像头等。

该协议定义了许多用于交换数据的标准 [消息](https://mavlink.io/en/messages/) 和 [微型服务（microservices）](https://mavlink.io/en/services/)（PX4 中用到了许多消息/服务，但不是全部）。

本教程介绍了如何为你自己新 "自定义" 的报文添加 PX4 支持。

> **Note** 本教程假定你在 `msg/ca_trajectory.msg` 文件中定义了一个名为 `ca_trajectory` 的 [自定义 uORB](../middleware/uorb.md) 消息，以及在 `mavlink/include/mavlink/v2.0/custom_messages/mavlink_msg_ca_trajectory.h` 文件中定义了一个名为 `ca_trajectory`的 自定义 MAVLink 消息。

## 创建自定义 MAVLink 消息

MAVlink 开发者指南介绍了如何定义新的消息并将其构建成指定的编程语言的库文件：

- [如何定义 MAVLink 消息（Messages）& 枚举（Enums）](https://mavlink.io/en/guide/define_xml_element.html)
- [生成 MAVLink 库文件](https://mavlink.io/en/getting_started/generate_libraries.html)

你需要为你的消息生成适用于 MAVLink 2 的 C 语言库文件。 只要你 [安装好了 MAVLink](https://mavlink.io/en/getting_started/installation.html) ，你可以使用如下命令执行此操作：

```sh
python -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 --output=generated/include/mavlink/v2.0 message_definitions/v1.0/custom_messages.xml
```

如果只是自己使用/测试，那么你只需要直接将生成的头文件拷贝到 **Firmware/mavlink/include/mavlink/v2.0** 文件夹下。

如果想让其他人可以更简单地测试你的修改，更好的做法则是将你生成的头文件加入 https://github.com/mavlink/c_library_v2 的一个分支中， PX4 开发者们则可以在开始编译构建前将 固件（Firmware）仓库中的子模块更新到你的分支上。

## 发送自定义MAVLink消息

此章节旨在说明：如何使用一条自定义uORB消息，并将其作为一条MAVLink消息发送出去。

Step1. 首先，在[mavlink_messages.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_messages.cpp)中添加自定义MAVLink消息和uORB消息的头文件：

```C
#include <uORB/topics/ca_trajectory.h>
#include <v2.0/custom_messages/mavlink.h>
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

Step3. 接下来，在[mavlink_messages.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_messages.cpp)的文件末尾，将这个消息流的类追加到`treams_list`。

```C
StreamListItem *streams_list[] = {
...
new StreamListItem(&MavlinkStreamCaTrajectory::new_instance, &MavlinkStreamCaTrajectory::get_name_static, &MavlinkStreamCaTrajectory::get_id_static),
nullptr
};
```

Step4. 最后，确保使能了此消息流：比如，添加如下内容至启动脚本文件（`-r`+n 命令可设置串流速率，`-u`命令可将MAVLink通道映射到UDP端口14556）

    mavlink stream -r 50 -s CA_TRAJECTORY -u 14556
    

> **Tip** 你可以使用 `uorb top [&lt;message_name&gt;]` 命令来实时验证你的消息是否被发布及消息的发布频率（详情请参阅： [uORB Messaging](../middleware/uorb.md#uorb-top-command)）。 这个方法还可以用来测试发布 uORB 主题的传入消息（对于其它类型的消息你可以在代码中使用 `printf` 然后再 SITL 仿真中进行测试）。
> 
> 要在 *QGroundControl* 中查看自定义消息，你需要 [使用你自己的 MAVLink 库重新构建该消息](https://dev.qgroundcontrol.com/en/getting_started/)，然后使用 [MAVLink Inspector Widget](https://docs.qgroundcontrol.com/en/app_menu/mavlink_inspector.html) （或者其它 MAVLink 工具） 验证是否收到该消息。

## 接收自定义MAVLink消息

此章节旨在说明：如何接收一条MAVLink消息，并将其发布至uORB。

Step1. 在[mavlink_receiver.h](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.h#L77)中添加自定义MAVLink消息和uORB消息的头文件：

```C
#include <uORB/topics/ca_trajectory.h>
#include <v2.0/custom_messages/mavlink_msg_ca_trajectory.h>
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

## 创建自定义MAVlink消息的备选方法

有时候需要创建一个内容尚未完全定义的自定义 MAVlink 消息。

例如，当使用 MAVlink 构建一个嵌入式设备与 PX4 的通讯接口时，设备与自动驾驶仪之间交换的消息的内容可能需要经过数次迭代之后才能稳定下来不发生变化。 在这种情况下重新生成 MAVlink 头文件并保证两个设备均使用同版本的协议会非常耗时且容易出错。

一个备选 - 也是临时的- 解决方案是重新使用（re-purpose）调试消息（debug messages）。 你可以发送一个以 `CA_TRAJ` 为字符串键，在`x`, `y` 和 `z` 字段中存放数据的 `DEBUG_VECT` 消息，而不需要创建一个自定义 MAVLink 消息 `CA_TRAJECTORY` 。 参阅 [这篇教程](../debug/debug_values.md) 以获取调试信息的更详细的使用方法。

> **Note** 此解决方案由于通过网络发送字符串并涉及字符串的比较，所以效率并不高。 此方法应仅用于开发！

## 常规信息

### 设置流速率（streaming rate）

有时候提高单个主题的流速率非常有用（例如在 QGC 中进行检查时）。 这可以通过在 shell 中输入如下命令实现：

```sh
mavlink stream -u <port number> -s <mavlink topic name> -r <rate>
```

你可以使用 `mavlink status` 来获取上述命令中的端口号，该指令的输出结果会包含 `transport protocol: UDP (&lt;port number&gt;)` 。 一个例子是：

```sh
mavlink stream -u 14556 -s OPTICAL_FLOW_RAD -r 300
```