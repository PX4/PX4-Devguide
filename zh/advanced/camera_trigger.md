---
translated_page: https://github.com/PX4/Devguide/blob/master/en/advanced/camera_trigger.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# 相机触发器

官网英文原文地址:http://dev.px4.io/advanced-camera-trigger.html

相机触发驱动器是为了让AUX端口发出一个脉冲来触发相机. 这个可以用于多个应用程序，包括航测和重建照片、同步多相机系统或者视觉惯性导航.

除了会发送一个脉冲, mavlink会传回一个序列号（当前所拍摄的图像的一个序列号）和拍摄的时间.
支持三种不同的模式:

- `触发` 1 就像一个基本的定时器，可以通过系统控制台分别设置启用和禁用 `相机触发使能` 或 `相机触发不使能`. 可以重复设置间隔时间，来执行相机触发.
- `触发模式` 2 用一个开关来定时何时曝光.
- `触发模式` 3 基于远程触发.每次超过设定的水平距离时都要触发拍摄
  . 然而，两个相机之间的最小时间间隔由设定的触发间隔时间决定.

在 `触发模式` 0 触发关闭.

想找到与相机触发模块有关的参数配置的完整列表，请参考 [参考](https://pixhawk.org/firmware/parameters#camera_trigger) 页.


> 如果这是你第一次启用相机触发应用程序, 记得要修改`触发模式` 参数为1, 2 或 3.


## 相机IMU设置同步的例程

在这个例程中, 我们将对IMU测量同步可视化数据的基础来建立一个立体视觉惯性导航系统（VINS）. 很明显, 这里的想法是IMU不参与测量，而是当到达设置的时间点时触发拍照，为什么提供准确的数据的迭代算法.

自驾仪和外设有不同的时钟(自驾仪的启动时间和UNIX外设的启动时间), 所以他们有不同的时间, 我们直接看时钟之间的时间偏移量. 这个偏移量是添加或从中减去mavlink消息的时间（如高分辨率的IMU）（如该译者在PX4同伴和mavlink的接收端）. 实际的同步算法是网络时间协议（NTP）算法的修改版本，使用指数移动平均平滑跟踪时间偏移.如果使用高宽带的车载连接会自动完成这种同步连接.
要获取同步的图像数据和惯性测量数据,我们就要把相机的触发信号输入引脚连接到自驾仪的GPIO引脚. 从中获取惯性测量的时间点数据和图像序列号记录并发送到配套的计算机（相机触发信息）,缓存这些数据和从相机获得的图像. 这些是匹配了序列号、时间点数据的图像.

下面的图表说明了事件的发生，必须以正确的时间点匹配我们的图像序列号。

{% mermaid %}
sequenceDiagram
  Note right of px4 : Time sync with mavros is done automatically
  px4 ->> mavros : Camera Trigger ready
  Note right of camera driver : Camera driver boots and is ready
  camera driver ->> mavros : mavros_msgs::CommandTriggerControl
  mavros ->> px4 : MAVLink::MAV_CMD_DO_TRIGGER_CONTROL
  loop Every TRIG_INTERVAL milliseconds
  px4 ->> mavros : MAVLink::CAMERA_TRIGGER
  mavros ->> camera driver : mavros_msgs::CamIMUStamp
  camera driver ->> camera driver : Match sequence number
  camera driver ->> camera driver : Stamp image and publish
end
{% endmermaid %}

### 步骤 1
 首先, 设置触发模式为模式1来使驱动程序等待开始命令并启动你的FCU来获取其余的参数.

### 步骤 2

对于这个例程的目的，我们将配置触发器操作一个Point Grey Firefly MV camer运行在30FPS.

- 触发间隔时间: 33.33 ms（）
- 触发信号极性: 0, 拉低电平
- 触发有效时间: 0.5 ms,无. 手动设置触发信号仅至少需要1微秒时间.
- 触发模式: 1, 因为我们希望我们的相机驱动程序准备好接收图像再开始触发. 这是正确处理序列号数据的必要条件和基础.
- 触发引脚: 12, 无.

### 步骤3

将你的相机连接到AUX接口的信号和地线.

### 步骤 4

你必须按照上面的顺序来修改你的驱动程序. 
公开的实现方法请参考[IDS 成像 UEye](https://github.com/ProjectArtemis/ueye_cam)
可兼容的相机列表请参考[兼容IEEE1394](https://github.com/andre-nguyen/camera1394) .
