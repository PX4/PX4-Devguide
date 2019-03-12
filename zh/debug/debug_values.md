---
translated_page: https://github.com/PX4/Devguide/blob/master/en/debug/debug_values.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# 发送调试的字符串/浮点数对

在软件开发期间通常需要输出单个重要的数字。
这时候就要用到MAVLink的`NAMED_VALUE`包。 

## Files

这个教程的代码可以从这里获取:

* [调试教程代码](https://github.com/PX4/Firmware/blob/master/src/examples/px4_mavlink_debug/px4_mavlink_debug.cpp)
* [使能教程应用](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v5/default.cmake) 通过取消掉板子中配置文件中对 mavlink debug app 的注释来使能这个应用。

配置一个调试发布（debug publication）只需要这个代码片段，先加入头文件：

<div class="host-code"></div>

```C
#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
```

然后公告（advertise）调试值主题 (不同发布名称（published names）一个公告就足够了).把这个放到主循环的前面:

<div class="host-code"></div>

```C
/* advertise debug value */
struct debug_key_value_s dbg = { .key = "velx", .value = 0.0f };
orb_advert_t pub_dbg = orb_advertise(ORB_ID(debug_key_value), &dbg);
```

在主循环里的发送甚至更简单：

<div class="host-code"></div>

```C
dbg.value = position[0];
orb_publish(ORB_ID(debug_key_value), pub_dbg, &dbg);
```

> **注意：** 多个调试消息必须在它们各自的发布之间有足够的时间以供Mavlink处理它们。这意味着代码必须在发布多个调试消息之间等待，或者在每个函数调用迭代上交替消息。


在QGroundControl中的结果实时绘图：

![](../../assets/gcs/qgc-debugval-plot.jpg)
