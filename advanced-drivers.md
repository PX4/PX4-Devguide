# Driver Development

The PX4 codebase uses a lightweight, unified driver abstraction layer: [DriverFramework](https://github.com/px4/DriverFramework). New drivers for POSIX and [QuRT](https://en.wikipedia.org/wiki/Qualcomm_Hexagon) are written against this framework.

<aside class="todo">
Legacy drivers for NuttX are based on the [Device](https://github.com/PX4/Firmware/tree/master/src/drivers/device) framework and will be ported to DriverFramework.
</aside>

## Core Architecture

PX4 is a [reactive system](concept-architecture.md) and uses pub/sub to transport messages. File handles are not required or used for the core operation of the system. Two main APIs are used:

  * The publish / subscribe system which has a file, network or shared memory backend depending on the system PX4 runs on
  * The global device registry, which allows to enumerate devices and get/set their configuration. This can be as simple as a linked list or map to the file system.

## Bringing up a new Platform

### NuttX

  * The start script is located in [ROMFS/px4fmu_common](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common)
  * The OS configuration is located in [nuttx-configs](https://github.com/PX4/Firmware/tree/master/nuttx-configs). The OS gets loaded as part of the application build.
  * The PX4 middleware configuration is located in [src/drivers/boards](https://github.com/PX4/Firmware/tree/master/src/drivers/boards). It contains bus and GPIO mappings and the board initialization code.
  * Drivers are located in [src/drivers](https://github.com/PX4/Firmware/tree/master/src/drivers)
  * Reference config: Running 'make px4fmu-v4_default' builds the FMUv4 config, which is the current NuttX reference configuration

### QuRT / Hexagon

  * The start script is located in [posix-configs/](https://github.com/PX4/Firmware/tree/master/posix-configs)
  * The OS configuration is part of the default Linux image (TODO: Provide location of LINUX IMAGE and flash instructions)
  * The PX4 middleware configuration is located in [src/drivers/boards](https://github.com/PX4/Firmware/tree/master/src/drivers/boards). TODO: ADD BUS CONFIG
  * Drivers are located in [DriverFramework](https://github.com/px4/DriverFramework)
  * Reference config: Running 'make qurt_eagle_release' builds the Snapdragon Flight reference config

## Device IDs

PX4 uses device IDs to identify individual sensors consistently across the system. These IDs are stored in the configuration parameters and used to match sensor calibration values, as well as to determine which sensor is logged to which logfile entry.

The order of sensors (e.g. if there is a `/dev/mag0` and an alternate `/dev/mag1`) is not determining priority - the priority is instead stored as part of the published uORB topic.

### Decoding example

For the example of three magnetometers on a system, use the flight log (.px4log) to dump the parameters. The three parameters encode the sensor IDs and `MAG_PRIME` identifies which magnetometer is selected as the primary sensor. Each MAGx_ID is a 24bit number and should be padded left with zeros for manual decoding.


```
CAL_MAG0_ID = 73225.0
CAL_MAG1_ID = 66826.0
CAL_MAG2_ID = 263178.0
CAL_MAG_PRIME = 73225.0
```

This is the external HMC5983 connected via I2C, bus 1 at address `0x1E`: It will show up in the log file as `IMU.MagX`.

```
# device ID 73225 in 24-bit binary:
00000001  00011110  00001 001

# decodes to:
HMC5883   0x1E    bus 1 I2C
```

This is the internal HMC5983 connected via SPI, bus 1, slave select slot 5. It will show up in the log file as `IMU1.MagX`.

```
# device ID 66826 in 24-bit binary:
00000001  00000101  00001 010

# decodes to:
HMC5883   dev 5   bus 1 SPI
```

And this is the internal MPU9250 magnetometer connected via SPI, bus 1, slave select slot 4. It will show up in the log file as `IMU2.MagX`.

```
# device ID 263178 in 24-bit binary:
00000100  00000100  00001 010

#decodes to:
MPU9250   dev 4   bus 1 SPI
```

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
