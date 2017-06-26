# Driver 개발

PX4는 가볍고 통합된 driver 추상 레이어를 사용합니다:
[DriverFramework](https://github.com/px4/DriverFramework). POSIX용 새로운 드라이버와 [QuRT](https://en.wikipedia.org/wiki/Qualcomm_Hexagon)가 이 프레임워크로 작성되었습니다.



> **Todo** Nuttx용 기존 driver는 [Device](https://github.com/PX4/Firmware/tree/master/src/drivers/device)을 기반으로 하며 DriverFramework로 포팅될 예정입니다.


## 핵심 아키텍쳐

PX4는 [reactive system](concept-architecture.md)으로 pub/sub을 사용해서 메시지를 전송합니다. 파일 핸들이 필요없으며 시스템을 운영하는 핵심입니다. 2개의 주요 API가 사용됩니다 :

  * publish / sucscribe 시스템으로 파일과 네트워크 혹은 백엔드로 공유 메모리를 가지는데 이는 PX4가 실행되는 시스템에 따라 달라집니다.
  * global device registry는 device를 열거하고 설정에 get/set 형태로 사용 가능합니다. 이는 linked list나 파일 시스템에 매핑처럼 간단하게도 가능합니다.

## 새로운 플랫폼 도입

### NuttX

  * 시작 스크립트의 위치는 [ROMFS/px4fmu_common](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common)입니다.
  * OS 설정은 [nuttx-configs](https://github.com/PX4/Firmware/tree/master/nuttx-configs)에 있습니다. OS는 application 빌드의 일부로 로드됩니다.
  * PX4 미들웨어 설정은 [src/drivers/boards](https://github.com/PX4/Firmware/tree/master/src/drivers/boards)에 있습니다. 여기에는 bus와 GPIO 매핑 그리고 보드 초기화 코드가 있습니다.
  * Driver는 [src/drivers](https://github.com/PX4/Firmware/tree/master/src/drivers)에 위치하고 있습니다.
  * Reference config: 'make px4fmu-v4_default'를 실행하면 FMUv4 config를 빌드합니다. 현재 NuttX를 참조하는 설정입니다.

### QuRT / Hexagon

  * 시작 스크립트는 [posix-configs/](https://github.com/PX4/Firmware/tree/master/posix-configs)에 있습니다.
  * OS 설정은 기본 Linux 이미지(TODO: Linux 이미지와 플래쉬 명령 제공)의 일부분 입니다.
  * PX4 미들웨어 설정은 [src/drivers/boards](https://github.com/PX4/Firmware/tree/master/src/drivers/boards)에 위치하고 있습니다. TODO: bus 설정 추가하기
  * Driver는 [DriverFramework](https://github.com/px4/DriverFramework)에 있습니다.
  * Reference config: 'make qurt_eagle_release'를 실행하면 Snapdragon Flight config를 빌드합니다.

## Device IDs

PX4는 시스템 내부에서 개별 센서를 식별하기 위해서 device ID를 사용합니다. ID는 설정 파라미터에 저장되며 센서 칼리브레이션 값과 매칭하는데 사용합니다. 또 어떤 센서가 logfile에 기록되어야할지를 결정합니다.

센서의 순서(예로 `/dev/mag0`이 기존에 있다면 `/dev/mag1`를 사용할 수 있다)가 우선순위를 결정하지는 않습니다. - 대신 우선순위는 published되는 uORB topic의 부분으로 저장됩니다.

### Decoding 예제


시스템에 있는 3개 지자기센서의 예제로 flight log (.px4log)를 사용해서 파라미터 정보를 확인합니다. 3개 파라미터는 센서 ID를 인코드하고 `MAG_PRIME`는 어느 지자기센서가 주센서로 선택되었는지 확인합니다. 각 MAGx_ID는 24 bit 수로 매뉴얼 디코딩시에 0을 padded left해줘야 합니다.


```
CAL_MAG0_ID = 73225.0
CAL_MAG1_ID = 66826.0
CAL_MAG2_ID = 263178.0
CAL_MAG_PRIME = 73225.0
```

이것은 I2C로 연결된 외부 HMC5983이고 bus 1의 주소는 `0x1E`입니다. `IMU.MagX`과 같은 로그 파일에서 볼 수 있습니다.

```
# device ID 73225 in 24-bit binary:
00000001  00011110  00001 001

# decodes to:
HMC5883   0x1E    bus 1 I2C
```

이것은 SPI로 연결된 내부 HMC5983이고 bus 1은 slave select slot 5입니다. `IMU1.MagX`과 같은 로그 파일에서 볼 수 있습니다.

```
# device ID 66826 in 24-bit binary:
00000001  00000101  00001 010

# decodes to:
HMC5883   dev 5   bus 1 SPI
```

이것은 SPI로 연결된 내부  MPU9250 지자기센서이며 bus 1은 slave select slot 4입니다. `IMU2.MagX`과 같은 로그 파일에서 볼 수 있습니다.

```
# device ID 263178 in 24-bit binary:
00000100  00000100  00001 010

#decodes to:
MPU9250   dev 4   bus 1 SPI
```

### Device ID 인코딩

device ID는 포캣을 따르는 24 bit 수 입니다. 첫번째 필드는 위의 예제의 디코딩에서 최하위 비트(least significant bits)가 됩니다.

```C
struct DeviceStructure {
  enum DeviceBusType bus_type : 3;
  uint8_t bus: 5;    // which instance of the bus type
  uint8_t address;   // address on the bus (eg. I2C address)
  uint8_t devtype;   // device class specific device type
};
```
`bus_type`는 다음에 따라 디코딩 :

```C
enum DeviceBusType {
  DeviceBusType_UNKNOWN = 0,
  DeviceBusType_I2C     = 1,
  DeviceBusType_SPI     = 2,
  DeviceBusType_UAVCAN  = 3,
};
```

그리고 `devtype`은 다음에 따라 디코딩 :

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
