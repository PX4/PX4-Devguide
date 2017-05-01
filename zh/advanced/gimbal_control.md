---
translated_page: https://github.com/PX4/Devguide/blob/master/en/advanced/gimbal_control.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# 设置云台控制


PX4包含一个通用的安装/云台控制驱动程序，具有不同的输入和输出模式。可以选择任何输入模式来驱动任何输出。

首先，确认驱动运行，运行 `vmount start`, 然后配置其参数。

## 参数
参数描述在[src/drivers/vmount/vmount_params.c](https://github.com/PX4/Firmware/blob/master/src/drivers/vmount/vmount_params.c)中。 其中，最重要的参数是输入 (`MNT_MODE_IN`)和输出 (`MNT_MODE_OUT`)模式。默认情况下，禁用输入。可以选择任何输入方式来驱动任何可用的输出。
如果选择了mavlink输入模式，则可以另外启动手动RC输入 (`MNT_MAN_CONTROL`)。只要没有收到mavlink消息，或mavlink明确请求RC模式，参数都是有用的。



### 配置云台混控器的AUX输出
云台使用控制组#2(请参阅混控和执行器篇)，这是混控器设置：

```
# roll
M: 1
O:      10000  10000      0 -10000  10000
S: 2 0  10000  10000      0 -10000  10000

# pitch
M: 1
O:      10000  10000      0 -10000  10000
S: 2 1  10000  10000      0 -10000  10000

# yaw
M: 1
O:      10000  10000      0 -10000  10000
S: 2 2  10000  10000      0 -10000  10000
```

将所需要的配置添加到主混控器或者辅混控器。

## 测试
驱动程序提供了一些简单的测试命令。需要先运行`vmount stop`停止。以下描述了SITL中的测试，但是这些命令行也可在真是设备上运行。

开启仿真(无需为此更改参数):
```
make posix gazebo_typhoon_h480
```
先确认已开桨，比如使用命令 `commander takeoff`，然后运行：
```
vmount test yaw 30
```
以控制云台。请注意，仿真中云台本身会增稳，因此，如果你发送mavlink命令，请将 `stabilize`  标志位置为false。

![Gazebo Gimbal Simulation](../../assets/gazebo/gimbal-simulation.png)



