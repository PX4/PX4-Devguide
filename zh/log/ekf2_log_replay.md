---
translated_page: https://github.com/PX4/Devguide/blob/master/en/log/ekf2_log_replay.md
translated_sha: 23bf0248facb1ab7d0dd58003c3234a95f479931
---

# EKF2日志重放

此页面说明如何使用真实的飞行日志上的重放(replay)功能来调整EKF2估计器的参数。

## 介绍
开发人员有可能需要对特别记录的数据进行重放以进行估计分析。 本页面的其余部分将说明必须设置哪些参数才能从此功能中受益，以及如何正确部署它。

## sdlog2 logger (.px4log)

### 准备工作

* 把参数 **SYS_LOGGER** 设为  sdlog2 (default) 然后重启飞控 (0 = sdlog2 and 1 = ulog).
* 把参数 **EKF2\_REC\_RPL** 设为 1. 这告诉估计器发布 **special** 重放信息给日志来记录
* 如果有的话, 把参数 **SDLOG\_PRIO\_BOOST** 设为如下格式 {0, 1, 2, 3}。 0 表示板载日志记录应用有默认值  \(低\) 的计划优先权。 低的计划优先权会导致日志信息的丢失。如果你大仙你的日志文件有由于跳过信息引起的空格，那么你可以增大这个系数到最大值3，有测试表明得最小值2才能避免丢失数据。

### 部署

一旦你使用上述设置并且有了真正的飞行日志（.px4log），则可以在PX4固件根目录中的使用以下命令来**run**运行重放：

```
make posix_sitl_replay replay logfile=<absolute_path_to_log_file>/my_log_file.px4log
```


一旦命令执行，检查终端的重放日志文件的位置和名称。

文件夹应该在

```
<path to Firmware>/build/posix_sitl_replay/src/firmware/posix/rootfs/
```
输出重放文件夹称为 **replay_replayed.px4log**，可以用来分析估计器的性能。


### 更改重放的调谐参数
当你第一次运行重放时，将使用实际飞行中的默认EKF2参数值生成replay_replayed.px4log文件。

之后，你可以改任何的EKF2系数值，这里要更改的文件 **replay\_params.txt**跟你的输出文件在同一个目录里。

例如，设置陀螺仪偏置的噪声值将需要以下行

```
EKF2_GB_NOISE 0.001
```
一旦某些EKF2参数被更改，可以使用[部署](#deployment)中给出的相同命令生成一个新的replay_replayed.px4log文件。


