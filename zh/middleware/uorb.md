# uORB消息机制

## 简介

uORB是一种用于进程间进行异步发布和订阅的消息机制API。

在[教程](../tutorials/tutorial_hello_sky.md)中可以学习通过C++如何使用uORB。

由于很多应用都是基于uORB的，因此在系统刚启动时uORB就自动运行了。uORB通过`uorb start`启动。可以使用`uorb test`进行单位测试。

## 添加新的主题(topic)

要想增加新的topic，你需要在`msg/`目录下创建一个新的`.msg` 文件并在`msg/CMakeLists.txt`下添加该文件。这样C/C++编译器自动在程序中添相应的代码。

可以先看看现有的`msg`文件了解下都支持那些类型。一个消息也可以嵌套在其他消息当中。

每一个生成的C/C++结构体中，会多出一个`uint64_t timestamp` 字段。这个变量用于将消息记录到日志当中。

为了在代码中使用"topic"需要添加头文件:

```
#include <uORB/topics/topic_name.h>
```

首先需要在文件`.msg`中，通过添加类似如下的一行代码,一个消息定义就可以用于多个独立的主题.

```
# TOPICS mission offboard_mission onboard_mission
```

> 【按】这里这一步将产生三个主题ID- mission、 offboard_mission 、以及 onboard_mission (第一个ID务必与.msg文件名相同)

然后在代码中, 把它们作为主题ID用:`ORB_ID(offboard_mission)`.

## 发布主题

在系统的任何地方都可以发布一个主题, 包括在中断上下文中(被`hrt_call`接口调用的函数). 但是, 公告(advertise)一个主题仅限于在中断上下文之外.

一个主题只能由同一个进程进行公告, 并作为其之后的发布(publish).

## 列出所有主题并进行监听

`接收者(listener)`命令仅在Pixracer（FMUv4）以及Linux/OS X上可用。

要列出所有主题, 先列出文件句柄:

```sh
ls /obj
```

要列出一个主题中的5个消息, 执行以下监听命令:

```sh
listener sensor_accel 5
```

得到的输出就是关于该主题的n次内容:

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

### uorb top Command
The command `uorb top` shows the publishing frequency of each topic in
real-time:

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
The columns are: topic name, multi-instance index, number of subscribers,
publishing frequency in Hz, number of lost messages (all subscribers combined), and
queue size.


## Multi-instance
uORB provides a mechanism to publish multiple independent instances of the same
topic through `orb_advertise_multi`. It will return an instance index to the
publisher. A subscriber will then have to choose to which instance to subscribe
to using `orb_subscribe_multi` (`orb_subscribe` subscribes to the first
instance).
Having multiple instances is useful for example if the system has several
sensors of the same type.

Make sure not to mix `orb_advertise_multi` and `orb_advertise` for the same
topic.

The full API is documented in
[src/modules/uORB/uORBManager.hpp](https://github.com/PX4/Firmware/blob/master/src/modules/uORB/uORBManager.hpp).

## Troubleshooting and common Pitfalls
The following explains some common pitfalls and corner cases:
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

