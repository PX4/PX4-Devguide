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

## Core Architecture

PX4 是一个 [reactive system ](../concept/architecture.md)，使用 [uORB](../middleware/uorb.md) 发布/订阅传输消息。 文件句柄不是必需或用于系统的核心操作。 使用了两个主要 API：

* Publish / subscribe 系统具有文件、网络或共享内存后端，具体取决于系统 PX4 运行。
* 全局设备注册表，可用于枚举设备并获取其配置。 这可以像链接列表或映射到文件系统一样简单。

## 设备ID

PX4 使用设备 ID 在整个系统中一致地识别单个传感器。 这些 ID 保存在配置参数中并且用于匹配传感器校准值，也用于决定那个传感器日志文件的入口。

The order of sensors (e.g. if there is a `/dev/mag0` and an alternate `/dev/mag1`) does not determine priority - the priority is instead stored as part of the published uORB topic.

### Decoding example

For the example of three magnetometers on a system, use the flight log (.px4log) to dump the parameters. The three parameters encode the sensor IDs and `MAG_PRIME` identifies which magnetometer is selected as the primary sensor. Each MAGx_ID is a 24bit number and should be padded left with zeros for manual decoding.

    CAL_MAG0_ID = 73225.0
    CAL_MAG1_ID = 66826.0
    CAL_MAG2_ID = 263178.0
    CAL_MAG_PRIME = 73225.0
    

This is the external HMC5983 connected via I2C, bus 1 at address `0x1E`: It will show up in the log file as `IMU.MagX`.

    # device ID 73225 in 24-bit binary:
    00000001  00011110  00001 001
    
    # decodes to:
    HMC5883   0x1E    bus 1 I2C
    

This is the internal HMC5983 connected via SPI, bus 1, slave select slot 5. It will show up in the log file as `IMU1.MagX`.

    # device ID 66826 in 24-bit binary:
    00000001  00000101  00001 010
    
    # decodes to:
    HMC5883   dev 5   bus 1 SPI
    

And this is the internal MPU9250 magnetometer connected via SPI, bus 1, slave select slot 4. It will show up in the log file as `IMU2.MagX`.

    # device ID 263178 in 24-bit binary:
    00000100  00000100  00001 010
    
    #decodes to:
    MPU9250   dev 4   bus 1 SPI
    

### Device ID Encoding

The device ID is a 24bit number according to this format. Note that the first fields are the least significant bits in the decoding example above.

```C
struct DeviceStructure {
  enum DeviceBusType bus_type : 3;
  uint8_t bus: 5;    // which instance of the bus type
  uint8_t address;   // address on the bus (eg. I2C address)
  uint8_t devtype;   // device class specific device type
};
```

The `bus_type` is decoded according to:

```C
enum DeviceBusType {
  DeviceBusType_UNKNOWN = 0,
  DeviceBusType_I2C     = 1,
  DeviceBusType_SPI     = 2,
  DeviceBusType_UAVCAN  = 3,
};
```

and `devtype` is decoded according to:

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