# 驱动开发

Nuttx 设备驱动基于 [Device](https://github.com/PX4/Firmware/tree/master/src/lib/drivers/device) 架构。

Linux 和 QuRT 驱动基于 [DriverFramework](https://github.com/px4/DriverFramework)。 PX4 目前正在升级以保证使用和 Nuttx 一样的驱动。

> **Note**目前（2017年12月），少量 linuxsw qurt i2c 驱动器已迁移（主要用于空速传感器）。 我们计划在即将发布的版本中迁移其余的驱动程序。

## 创建驱动程序

PX4 几乎只消耗来自 [uORB](../middleware/uorb.md) 的数据。 常见外设类型的驱动程序必须发布正确的 uORB 消息（例如: 陀螺仪、加速度计、压力传感器等）。

创建新驱动程序的最佳方法是从类似的驱动程序作为模板开始（请参阅 [src/drivers](https://github.com/PX4/Firmware/tree/master/src/drivers)）。

> **Tip** 有关使用特定 ito 总线和传感器的更多详细信息，请参见 [传感器和执行器总线](../sensor_bus/README.md) 部分。

<span></span>

> **Note** 发布正确的 uORB 主题是驱动程序 *必须* 遵循的唯一模式。

## 核心架构

PX4 是一个 [reactive system ](../concept/architecture.md)，使用 [uORB](../middleware/uorb.md) 发布/订阅传输消息。 文件句柄不是必需或用于系统的核心操作。 使用了两个主要 API：

* Publish / subscribe 系统具有文件、网络或共享内存后端，具体取决于系统 PX4 运行。
* 全局设备注册表，可用于枚举设备并获取其配置。 这可以像链接列表或映射到文件系统一样简单。

## 设备ID

PX4 使用设备 ID 在整个系统中一致地识别单个传感器。 这些 ID 保存在配置参数中并且用于匹配传感器校准值，也用于决定那个传感器日志文件的入口。

传感器序列（比如，如果有一个 `/dev/mag0` 和一个备用的 `/dev/mag1`）并不决定优先级 — 优先级作为已发布的 uORB 主题的一部分存储。

### 解码示例

对于系统上三个磁强计的示例，请使用飞行日志 (. px4log) 转储参数。 这三个参数解码传感器的 ID， 并且 `MAG_PRIME` 区分那个磁力计作为主传感器。 每个 MAGx_ID 是 24 bit 的数，手动解码的话高位补 0。

    CAL_MAG0_ID = 73225.0
    CAL_MAG1_ID = 66826.0
    CAL_MAG2_ID = 263178.0
    CAL_MAG_PRIME = 73225.0
    

这是通过 I2C 总线1 的外部 HMC5983 连接在地址 `0x1E`：会在日志文件中以 `IMU.MagX` 格式显示出来。

    # device ID 73225 in 24-bit binary:
    00000001  00011110  00001 001
    
    # decodes to:
    HMC5883   0x1E    bus 1 I2C
    

这是内部 HMC5983 连接通过 SPI 总线 1，所以选择插槽5。 它将以 `IMU1.MagX` 显示在日志文件中。

    # device ID 66826 in 24-bit binary:
    00000001  00000101  00001 010
    
    # decodes to:
    HMC5883   dev 5   bus 1 SPI
    

这是通过 spi、总线1、从选择插槽4连接的内部 MPU9250 磁强计。 它将以 `IMU2.MagX` 显示在日志文件中。

    # device ID 263178 in 24-bit binary:
    00000100  00000100  00001 010
    
    #decodes to:
    MPU9250   dev 4   bus 1 SPI
    

### 设备 ID 编码

根据此格式，设备 ID 是一个24bit 数字。 请注意，在上面的解码示例中，第一个字段是最不重要的位。

```C
struct DeviceStructure {
  enum DeviceBusType bus_type : 3;
  uint8_t bus: 5;    // which instance of the bus type
  uint8_t address;   // address on the bus (eg. I2C address)
  uint8_t devtype;   // device class specific device type
};
```

`bus_type` 是根据以下情况解码的：

```C
enum DeviceBusType {
  DeviceBusType_UNKNOWN = 0,
  DeviceBusType_I2C     = 1,
  DeviceBusType_SPI     = 2,
  DeviceBusType_UAVCAN  = 3,
};
```

`devtype` 是根据以下情况解码的：

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