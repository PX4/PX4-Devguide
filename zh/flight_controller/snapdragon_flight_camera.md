---
translated_page: https://github.com/PX4/Devguide/blob/master/en/flight_controller/snapdragon_flight_camera.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# 将相机用于骁龙飞控

骁龙飞控具有向下的灰度级相机，可用于基于光流的位置稳定和前置RGB相机。  [snap_cam](https://github.com/PX4/snap_cam) 仓库提供了一种运行和流式传输不同相机并计算光流的方法。

除了相机外，光流需要向下的距离传感器。在这里，讨论了TeraRanger One的使用。

## 光流
光流在应用处理器上计算并通过Mavlink发送到PX4。.
根据其自述文件中的说明克隆并编译[snap_cam](https://github.com/PX4/snap_cam)

以root身份运行光流应用程序:
```
optical_flow -n 50 -f 30
```

光流应用需要来自PX4的IMU Mavlink消息. 您可能需要添加一个额外的Mavlink实例到PX4通过添加以下到您的 `mainapp.config`:
```
mavlink start -u 14557 -r 1000000 -t 127.0.0.1 -o 14558
mavlink stream -u 14557 -s HIGHRES_IMU -r 250
```

### TeraRanger One setup
为了连接 TeraRanger One (TROne) 到骁龙飞控, 必须使用TROne I2C适配器，TROne必须使用供应商的I2C固件

THRO通过定制的DF13 4针至6针电缆连接到骁龙飞控. 我们建议使用连接器J15（USB旁边的那个），因为所有其他的已经在使用（RC，ESCs，GPS）。 接线如下:

| 4 pin | <-> | 6 pin |
| -- | -- | -- |
| 1 |  | 1 |
| 2 |  | 6 |
| 3 |  | 4 |
| 4 |  | 5 |
THRO必须用10 - 20V的电源供电.
