# 드라이버 개발

PX4 장치 드라이버는 [Device](https://github.com/PX4/Firmware/tree/master/src/lib/drivers/device) 프레임워크에 기반합니다.

## 드라이버 만들기

PX4는 [uORB](../middleware/uorb.md) 데이터만을 유일하게 가져옵니다. 일반 주변기기 형식의 드라이버는 올바른 uORB 메시지를 내보내야합니다(예: 각가속계, 가속계, 압력계 등)

새 드라이버를 만드는 가장 바람직한 접근법은 코드 서식을 통해 유사 드라이버로 시작하는 방식입니다([src/drivers](https://github.com/PX4/Firmware/tree/master/src/drivers) 참고).

> **Tip** 특정 입출력 버스와 센서와 동작하기 위한 자세한 정보는 [센서와 액츄에이터 버스](../sensor_bus/README.md)절을 참고하십시오.

<span></span>

> **Note** 드라이버가 *반드시* 따라야 할 정규 동작은 올바른 uORB 토픽의 전송입니다.

## 핵심 아키텍처

PX4는 [반응형 시스템](../concept/architecture.md)이며 메세지 송수신시 [uORB](../middleware/uorb.md) 송신/가입 방식을 활용합니다. 시스템의 핵심부 처리에 있어 파일 핸들을 활용하거나 필요로 하지 않습니다. 주요 API는 두가지를 활용합니다:

* Pub/Sub 시스템은 PX4가 실행되는 시스템에 의존하는 네트워크나 공유메모리 백엔드가 있습니다.
* 글로벌 장치 레지스트리를 통해 디바이스 목록과 그 설정을 get/set할 수 있습니다. 이것은 링크리스트처럼 간단하며, 파일시스템에 매핑할 수도 있습니다.

## 디바이스 ID

PX4 uses device IDs to identify individual sensors consistently across the system. These IDs are stored in the configuration parameters and used to match sensor calibration values, as well as to determine which sensor is logged to which logfile entry.

The order of sensors (e.g. if there is a `/dev/mag0` and an alternate `/dev/mag1`) does not determine priority - the priority is instead stored as part of the published uORB topic.

### 디코딩 예제

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
    

### 디바이스 ID 인코딩

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

## Debugging

For general debugging topics see: [Debugging/Logging](../debug/README.md).

### Verbose Logging

Drivers (and other modules) output minimally verbose logs strings by default (e.g. for `PX4_DEBUG`, `PX4_WARN`, `PX4_ERR`, etc.).

Log verbosity is defined at build time using the `RELEASE_BUILD` (default), `DEBUG_BUILD` (verbose) or `TRACE_BUILD` (extremely verbose) macros.

Change the logging level using `COMPILE_FLAGS` in the driver `px4_add_module` function (**CMakeLists.txt**). The code fragment below shows the required change to enable DEBUG_BUILD level debugging for a single module or driver.

    px4_add_module(
        MODULE templates__module
        MAIN module
    

        COMPILE_FLAGS
            -DDEBUG_BUILD
    

        SRCS
            module.cpp
        DEPENDS
            modules__uORB
        )
    

> **Tip** Verbose logging can also be enabled on a per-file basis, by adding `#define DEBUG_BUILD` at the very top of a .cpp file (before any includes).