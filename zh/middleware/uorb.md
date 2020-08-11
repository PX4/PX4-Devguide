# uORB 消息

## 简介

The uORB is an asynchronous `publish()` / `subscribe()` messaging API used for inter-thread/inter-process communication.

查看 [教程](../apps/hello_sky.md) 以了解如何在 C++ 中使用它。

uORB 会在启动时自动启动，因为许多应用程序都依赖于它。 它以 `uorb start</0 > 开头。 单元测试可以从 <code>uorb_tests` 开始。

## 添加新 Topic（主题）

New uORB topics can be added either within the main PX4/Firmware repository, or can be added in an out-of-tree message definitions. For information on adding out-of-tree uORB message definitions, please see [this section](../advanced/out_of_tree_modules.md#uorb_message_definitions).

To add a new topic, you need to create a new **.msg** file in the `msg/` directory and add the file name to the `msg/CMakeLists.txt` list. From this, the needed C/C++ code is automatically generated.

查看支持类型的现有 `msg` 文件。 A message can also be used nested in other messages.

对于每个生成的 C/C + 结构，将添加一个字段 `uint64_t timestamp `。 This is used for the logger, so make sure to fill it in when publishing the message.

若要在代码中使用该主题，请包括头文件：

    #include <uORB/topics/topic_name.h>
    

By adding a line like the following in the `.msg` file, a single message definition can be used for multiple independent topics:

    # TOPICS mission offboard_mission onboard_mission
    

然后在代码中，将它们用作主题 id: `ORB_ID(offboard_mission)`。

## 发布

Publishing a topic can be done from anywhere in the system, including interrupt context (functions called by the `hrt_call` API). However, advertising a topic is only possible outside of interrupt context. A topic has to be advertised in the same process as it's later published.

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

> **Tip** 在基于 NuttX 的系统上（如 Pixhawk， Pixracer等），监听器可以用 *QGroundControl* 内部的 MAVLink 终端监视传感器的值和其他主题。 之所以是非常有用的调试工具是因为可以在 QGC 上通过无线连接（比如飞机在飞行过程中）。 有关详细信息，请参阅 [传感器/主题调试 ](../debug/sensor_uorb_topic_debugging.md)。

### uorb top 命令

uorb top 命令实时显示每个主题的发布频率。

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

## 多实例

uORB provides a mechanism to publish multiple independent instances of the same topic through `orb_advertise_multi`. It will return an instance index to the publisher. A subscriber will then have to choose to which instance to subscribe to using `orb_subscribe_multi` (`orb_subscribe` subscribes to the first instance). Having multiple instances is useful for example if the system has several sensors of the same type.

请确保不要为同一主题混合 `orb_advertise_multi` 和 `orb_advertise`。

完整的 API 记录在 [src/modules/uORB/uORBManager.hpp](https://github.com/PX4/Firmware/blob/master/src/modules/uORB/uORBManager.hpp) 中。

## Message/Field Deprecation {#deprecation}

As there are external tools using uORB messages from log files, such as [Flight Review](https://github.com/PX4/flight_review), certain aspects need to be considered when updating existing messages:

- Changing existing fields or messages that external tools rely on is generally acceptable if there are good reasons for the update. In particular for breaking changes to *Flight Review*, *Flight Review* must be updated before code is merged to `master`.
- In order for external tools to reliably distinguish between two message versions, the following steps must be followed: 
  - Removed or renamed messages must be added to the `deprecated_msgs` list in [msg/CMakeLists.txt](https://github.com/PX4/Firmware/blob/master/msg/CMakeLists.txt#L157) and the **.msg** file needs to be deleted.
  - Removed or renamed fields must be commented and marked as deprecated. For example `uint8 quat_reset_counter` would become `# DEPRECATED: uint8 quat_reset_counter`. This is to ensure that removed fields (or messages) are not re-added in future.
  - In case of a semantic change (e.g. the unit changes from degrees to radians), the field must be renamed as well and the previous one marked as deprecated as above.