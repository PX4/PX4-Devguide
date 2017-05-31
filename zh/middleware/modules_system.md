# 模块参考：系统
## logger
源代码: [modules/logger](https://github.com/PX4/Firmware/tree/master/src/modules/logger)


### 说明
系统日志，记录一组可配置的uORB主题和系统打印消息（`PX4_WARN`和`PX4_ERR`）到ULog文件。这些可用于系统和飞行性能评估，调整，再现和意外分析。

它支持2个后端：
- 文件：将ULog文件写入文件系统（SD卡）
- MAVLink：通过MAVLink向客户端传输ULog数据（客户端必须支持此功能）

两个后端可以同时启用和使用。

### 实现
实现使用两个线程：
- 主线程以固定速率运行（如果以-p参数启动的话，则轮询主题），并检查数据更新
- 写入线程，将数据写入文件

两个线程之间有一个可配置大小的写入缓存区。写入缓存应该足够大，以避免数据丢失。

### 示例
立即启动日志的典型用法
```
logger start -e -t
```

如果已经在运行状态：
```
logger on
```

### 用法
```
logger <命令> [参数...]
 命令:
   start
     [-m <val>]  后端模式
                 可选: file|mavlink|all, 缺省: all
     [-e]        启动后立即开始记录，直到上锁(否则只在解锁后开始记录)
     [-f]        记录直到关闭为止(包含 -e)
     [-t]        使用日期/时间命名日志目录和文件
     [-r <val>]  记录速率，单位：Hz，值为0则不限速
                 缺省: 280
     [-b <val>]  记录缓存大小，单位：KiB
                 缺省: 12
     [-q <val>]  mavlink模式下uORB队列大小
                 缺省: 14
     [-p <val>]  轮询主题而不是以固定速率运行(记录速率和主题间隔将被忽略)
                 可选: <topic_name>

   on            立即开始记录，覆盖解锁(日志系统必须在运行中)

   off           立即停止记录，覆盖锁定(日志系统必须在运行中)

   stop

   status        打印状态信息
```
## send_event
源代码: [modules/events](https://github.com/PX4/Firmware/tree/master/src/modules/events)


### 说明
后台进程在低优先级工作队列上定期运行以执行内部处理任务。目前只负责温度校准。

这些任务可以通过CLI或uORB主题（来自MAVLink的vehicle_command等）启动。

### 用法
```
send_event <命令> [参数...]
 命令:
   start         启动后台任务

   temperature_calibration 运行温度校准程序
     [-g]        校准陀螺仪
     [-a]        校准加速度计
     [-b]        校准磁罗盘(如果没有给出这些参数，将会校准所有)

   stop

   status        打印状态信息
```
