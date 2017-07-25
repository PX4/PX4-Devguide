---
translated_page: https://github.com/PX4/Devguide/blob/master/en/middleware/uorb.md
translated_sha: 18f5865bf5265934136cf5d18f838203c3db2100
---

# uORB消息机制

## 简介

uORB是一种用于线程间/进程间进行异步发布-订阅的消息机制的应用程序接口（API）。

在[这个教程](../tutorials/tutorial_hello_sky.md)中可以学习通过C++如何使用uORB。

由于很多应用都是基于uORB的，因此在系统刚启动时uORB就自动运行了。uORB通过`uorb start`启动。可以使用`uorb test`进行单位测试。

## 添加新的话题(topic)

要想增加新的topic，你需要在`msg/`目录下创建一个新的`.msg` 文件，并在`msg/CMakeLists.txt`下添加该文件名。这样会自动生成所需的C / C ++代码

可以先看看现有的`msg`文件所支持的类型。一个消息也可以嵌套在其他消息当中。

每一个生成的C/C++结构体中，需要添加一个`uint64_t timestamp` 时间戳字段。该字段用于记录器（logger），因此确保在发布消息时一定要添加它。

为了在代码中使用"topic"，需要添加头文件:

```
#include <uORB/topics/topic_name.h>
```

通过在`.msg`文件中，添加类似如下的一行代码,一个消息定义就可以用于多个独立的topic中：

```
# TOPICS mission offboard_mission onboard_mission
```

> 【按】这里这一步将产生三个topic ID- mission、 offboard_mission 以及 onboard_mission (第一个ID务必与.msg文件名相同)

然后在代码中, 通过topic ID:`ORB_ID(offboard_mission)`来使用这个topic.

## 发布话题

在系统的任何地方都可以发布（publish）一个话题, 包括在中断上下文中(被`hrt_call`接口调用的函数). 但是, 公告(advertise)一个话题仅限于在中断上下文之外. 一个话题必须同它随后发布的同一进程中公告。一个话题必须在它随后发布的相同进程中进行公告。

## 列出话题并进行监听

> **注意** `监听器(listener)`命令仅在Pixracer（FMUv4）以及Linux/OS X上可用。

要列出所有话题, 先列出文件句柄:

```sh
ls /obj
```

要列出一个话题中的5个消息, 执行以下监听命令:

```sh
listener sensor_accel 5
```

得到的输出就是关于该话题的n次内容:

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

> **提示** 在基于NuttX的系统(Pixhawk, Pixracer等)， `listener`命令可从地面站*QGroundControl* MAVLink控制台调用，来监听传感器数值和其他话题。 这是一个强大的调试工具，因为QGC通过无线链路连接时也可以使用它（例如，当无人机在飞行过程中）。更多信息可以看[Sensor/Topic Debugging](../debug/sensor_uorb_topic_debugging.md).


### uorb up 命令
`uorb top` 命令可以实时显示每个话题的发布频率：

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
每列分别是：话题名，多实例索引，订阅者数，发布频率(Hz)，丢失消息数（所有订阅者合并显示），队列大小。


## 多实例（Multi-instance）
uORB提供一种通过 `orb_advertise_multi` 发布同一话题的多个实例的机制。它将向发布者（publisher）返回一个实例索引。一个订阅者（subscriber）必须用 `orb_subscribe_multi` (`orb_subscribe` ，订阅第一个实例)来选择订阅哪个实例。对于一个具有多个相同类型传感器的系统，这种多实例机制非常有用。

对于同一个话题，确保不要将 `orb_advertise_multi` 和 `orb_advertise` 混淆。

完整的API文档可见[src/modules/uORB/uORBManager.hpp](https://github.com/PX4/Firmware/blob/master/src/modules/uORB/uORBManager.hpp).

## 故障排除和常见问题
以下列出一些常见的问题和几个极端情况：
- The topic is not published: make sure the `ORB_ID()`'s of each call match. It
  is also important that `orb_subscribe` and `orb_unsubscribe` are **called from
  the same task** as `orb_publish`. This applies to `px4_task_spawn_cmd()`, but
  also when using work queues (`work_queue()`).
- Make sure to clean up: use `orb_unsubscribe` and `orb_unadvertise`.
- A successful `orb_check()` or `px4_poll()` call requires an `orb_copy()`,
  otherwise the next poll will return immediately.
- It is perfectly ok to call `orb_subscribe` before anyone advertised the topic.
- `orb_check()` and `px4_poll()` will only return true for publications that are
  done after `orb_subscribe()`. This is important for topics that are not
  published regularly. If a subscriber needs the previous data, it should just
  do an unconditional `orb_copy()` right after `orb_subscribe()` (Note that
  `orb_copy()` will fail if there is no advertiser yet).

