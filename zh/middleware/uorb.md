# uORB Messaging

## 简介

uORB 是用于进程间通信的异步 `publish()`/`subscribe()` 消息传递 API。

查看 [教程](../apps/hello_sky.md) 以了解如何在 C++ 中使用它。

uORB 会在启动时自动启动，因为许多应用程序都依赖于它。 它以 `uorb start</0 > 开头。 单元测试可以从 <code>uorb_tests` 开始。

## 添加新 Topic（主题）

可以在主 PX4/Firmware 存储库中添加新的 uORB 主题，也可以在树外消息定义中添加。 有关添加树外 uORB 消息定义的信息，请参阅 [本节](../advanced/out_of_tree_modules.md#uorb_message_definitions)。

若要添加新主题，需要在 `msg/` 目录中创建一个新的 **.msg** 文件，并将文件名添加到 `msg/CMakeLists.txt` 列表中。 由此，将自动生成所需的 C/C++ 代码。

查看支持类型的现有 `msg` 文件。 消息还可以在其他消息中嵌套使用。

对于每个生成的 C/C + 结构，将添加一个字段 `uint64_t timestamp `。 此用于记录日志，因此请确保在发布时填充数据。

若要在代码中使用该主题，请包括头文件：

    #include <uORB/topics/topic_name.h>
    

通过在 `.msg` 文件中添加如下内容的行，可以将一条消息定义用于多个独立主题：

    # TOPICS mission offboard_mission onboard_mission
    

然后在代码中，将它们用作主题 id: `ORB_ID(offboard_mission)`。

## 发布

发布主题可以在系统中的任何位置完成，包括中断上下文（由 `hrt_call` API 调用的函数）。 但是，仅在中断上下文之外才能为主题做广播。 一个主题必须与以后发布的过程相同。

## 主题列表和监听（Listener）

> **Note** `listener` 命令仅适用于 Pixracer (FMUv4) 和 Linux/OS X。

要列出所有主题，列出文件句柄：

```sh
ls /obj
```

要监听五条信息中的一个主题内容，运行监听器：

```sh
listener sensor_accel 5
```

输出主题内容如下：

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

## Troubleshooting and common Pitfalls

The following explains some common pitfalls and corner cases:

- The topic is not published: make sure the `ORB_ID()`'s of each call match. It is also important that `orb_subscribe` and `orb_unsubscribe` are **called from the same task** as `orb_check` and `orb_copy`. This applies to `px4_task_spawn_cmd()`, but also when using work queues (`work_queue()`).
- Make sure to clean up: use `orb_unsubscribe` and `orb_unadvertise`.
- A successful `orb_check()` or `px4_poll()` call requires an `orb_copy()`, otherwise the next poll will return immediately.
- It is perfectly ok to call `orb_subscribe` before anyone advertised the topic.
- `orb_check()` and `px4_poll()` will only return true for publications that are done after `orb_subscribe()`. This is important for topics that are not published regularly. If a subscriber needs the previous data, it should just do an unconditional `orb_copy()` right after `orb_subscribe()` (note that `orb_copy()` will fail if there is no advertiser yet).