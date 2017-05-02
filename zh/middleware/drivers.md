---
translated_page: https://github.com/PX4/Devguide/blob/master/en/middleware/drivers.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# 驱动框架


PX4的代码库使用一个轻量级的，统一的驱动抽象层：[DriverFramework](https://github.com/px4/DriverFramework). 
POSIX和 [QuRT](https://en.wikipedia.org/wiki/Qualcomm_Hexagon)的驱动写入这个驱动框架当中。

> **Todo** 旧的NuttX驱动是基于[设备](https://github.com/PX4/Firmware/tree/master/src/drivers/device) 架构的，以后将会移植到驱动框架之中。


## 核心架构
PX4 是一个反应式系统[reactive system](../concept/architecture.md) ，使用订阅/发布来传递消息。文件句柄是不被操作系统的核心所需要或者使用。主要使用了以下两个API：

- 发布/订阅系统，该系统拥有一个文件，网络或者共享内存，其依靠于PX4后台运行。
- 全局驱动注册器，它允许枚举设备和获取/设置这些设备参数。这个可以很简单的作为一个链表或者文件系统地图。

## 一个新的平台
### NuttX
- 启动脚本位于[ROMFS/px4fmu_common](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common)
- 系统配置文件位于[nuttx-configs](https://github.com/PX4/Firmware/tree/master/nuttx-configs). 作为应用的一部分被构建并被操作系统加载。
- PX4中间件配置位于[src/drivers/boards](https://github.com/PX4/Firmware/tree/master/src/drivers/boards).其中包括总线和GPIO映射还有硬件平台初始化代码。
- 驱动位于[src/drivers](https://github.com/PX4/Firmware/tree/master/src/drivers)
- 参考配置:运行使px4fmu-v4_default构建FMUv4配置,这是当前NuttX参考配置。

### QuRT / Hexagon
- 启动脚本位于 [posix-configs/](https://github.com/PX4/Firmware/tree/master/posix-configs)
- 系统配置文件模式作为Linux映射的一部分（备注：提供 本地的LINUX IMAGE和flash指令）
- PX4中间件配置位于[src/drivers/boards](https://github.com/PX4/Firmware/tree/master/src/drivers/boards)。备注：增加总线配置。
- 驱动位于[DriverFramework](https://github.com/px4/DriverFramework)
- 参考配置：运行'make qurt_eagle_release'构建Snapdragon飞行参考配置。


## 驱动ID
PX4使用驱动ID将独立传感器贯穿于整个系统。这些ID存储于配置参数中，用于匹配传感器校正值，以及决定哪些传感器被记录到log中。

传感器的顺序（例如一个是`/dev/mag0`，另一个是`/dev/mag1`）于优先级是不挂钩的，优先级实际是在发布uORB topic时确定的。

### 举个例子

有关系统上三个磁力计的示例，使用飞行日志（.px4log）转存变量。三个参数对传感器ID进行编码，MAG_PRIME识别哪个磁力计被选为主传感器。每一个MAGx_ID是一个24bit数值，左面手工填零补充。

```
CAL_MAG0_ID = 73225.0
CAL_MAG1_ID = 66826.0
CAL_MAG2_ID = 263178.0
CAL_MAG_PRIME = 73225.0
```
通过I2C连接的外部HMC5983，总线1，地址0x1E：在log中以`IMU.MagX`显示。

```
# device ID 73225 in 24-bit binary:
00000001  00011110  00001 001

# decodes to:
HMC5883   0x1E    bus 1 I2C
```

通过SPI连接的内部HMC5983，总线1，选择slot5。在log中以`IMU1.MagX`显示。

```
# device ID 66826 in 24-bit binary:
00000001  00000101  00001 010

# decodes to:
HMC5883   dev 5   bus 1 SPI
```

以及通过SPI总线连接的内部MPU9250磁力计，总线1，从设备选择slot4。在log中以`IMU2.MagX`显示。

```
# device ID 263178 in 24-bit binary:
00000100  00000100  00001 010

#decodes to:
MPU9250   dev 4   bus 1 SPI
```

### 设备ID编码
根据此格式，设备ID是一个24位的数字。注意，第一字段是上述解码示例中的最低有效位。

```C
struct DeviceStructure {
  enum DeviceBusType bus_type : 3;
  uint8_t bus: 5;    // which instance of the bus type
  uint8_t address;   // address on the bus (eg. I2C address)
  uint8_t devtype;   // device class specific device type
};
```
这里`bus_type` 按以下方式解码：

```C
enum DeviceBusType {
  DeviceBusType_UNKNOWN = 0,
  DeviceBusType_I2C     = 1,
  DeviceBusType_SPI     = 2,
  DeviceBusType_UAVCAN  = 3,
};
```

`devtype` 按以下方式解码：

```C
#define DRV_MAG_DEVTYPE_HMC5883  0x01
#define DRV_MAG_DEVTYPE_LSM303D  0x02
#define DRV_MAG_DEVTYPE_ACCELSIM 0x03
#define DRV_MAG_DEVTYPE_MPU9250  0x04
#define DRV_ACC_DEVTYPE_LSM303D  0x11
#define DRV_ACC_DEVTYPE_BMA180   0x12
#define DRV_ACC_DEVTYPE_MPU6000  0x13
#define DRV_ACC_DEVTYPE_ACCELSIM 0x14
#define DRV_ACC_DEVTYPE_GYROSIM  0x15
#define DRV_ACC_DEVTYPE_MPU9250  0x16
#define DRV_GYR_DEVTYPE_MPU6000  0x21
#define DRV_GYR_DEVTYPE_L3GD20   0x22
#define DRV_GYR_DEVTYPE_GYROSIM  0x23
#define DRV_GYR_DEVTYPE_MPU9250  0x24
#define DRV_RNG_DEVTYPE_MB12XX   0x31
#define DRV_RNG_DEVTYPE_LL40LS   0x32
```
