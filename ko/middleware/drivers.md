!REDIRECT "https://docs.px4.io/master/ko/middleware/drivers.html"

# 드라이버 개발

PX4 device drivers are based on the [Device](https://github.com/PX4/PX4-Autopilot/tree/master/src/lib/drivers/device) framework.

## 드라이버 만들기

PX4는 [uORB](../middleware/uorb.md) 데이터만을 유일하게 가져옵니다. 일반 주변기기 형식의 드라이버는 올바른 uORB 메시지를 내보내야합니다(예: 각가속계, 가속계, 압력계 등)

The best approach for creating a new driver is to start with a similar driver as a template (see [src/drivers](https://github.com/PX4/PX4-Autopilot/tree/master/src/drivers)).

> **Tip** 특정 입출력 버스와 센서와 동작하기 위한 자세한 정보는 [센서와 액츄에이터 버스](../sensor_bus/README.md)절을 참고하십시오.

<span></span>

> **Note** 드라이버가 *반드시* 따라야 할 정규 동작은 올바른 uORB 토픽의 전송입니다.

## 핵심 아키텍처

PX4는 [반응형 시스템](../concept/architecture.md)이며 메세지 송수신시 [uORB](../middleware/uorb.md) 송신/가입 방식을 활용합니다. 시스템의 핵심부 처리에 있어 파일 핸들을 활용하거나 필요로 하지 않습니다. 주요 API는 두가지를 활용합니다:

* Pub/Sub 시스템은 PX4가 실행되는 시스템에 의존하는 네트워크나 공유메모리 백엔드가 있습니다.
* 글로벌 장치 레지스트리를 통해 디바이스 목록과 그 설정을 get/set할 수 있습니다. 이것은 링크리스트처럼 간단하며, 파일시스템에 매핑할 수도 있습니다.

## 장치 ID

PX4는 장치 ID를 시스템의 개별 센서를 주기적으로 식별하는 용도로 활용합니다. 이 ID는 설정 매개변수에 저장하며, 어떤 로그 파일 항목에 어떤 센서 내용을 기록할 지 결정하듯, 센서 보정값을 일치할 때 활용하기도 합니다.

센서 순서는(예: `/dev/mag0` 와 `/dev/mag1`가 있을 때) 우선 순위를 결정하지 않습니다. 우선순위는 uORB 토픽을 내보낼 일부로 저장할 뿐입니다).

### 디코딩 예제

예를 들어 세개의 지자계 센서가 있을 때 덤프에 넣으려는 매개변수 값을 비행 로그(.px4log)에서 활용합니다. 세 매개변수는 센서 ID와 어떤 지자계 센서를 첫번째 센서로 선택할 지 식별할 `MAG_PRIME`을 인코딩 합니다. 각각의 MAGx_ID는 24비트 숫자이며 직접 인코딩을 진행할 경우 상위 비트(좌측)을 0으로 채워야 합니다.

    CAL_MAG0_ID = 73225.0
    CAL_MAG1_ID = 66826.0
    CAL_MAG2_ID = 263178.0
    CAL_MAG_PRIME = 73225.0
    

I2C로 연결한 외부 HMC5983 칩입니다. `0x1E` 주소의 버스 1번입니다. 로그 파일에서 `IMU.MagX`로 나타납니다.

    # device ID 73225 in 24-bit binary:
    00000001  00011110  00001 001
    
    # decodes to:
    HMC5883   0x1E    bus 1 I2C
    

SPI로 연결한 내부 HMC5983 칩입니다. 버스 1번에서 슬롯 5번을 선택한 하위노드입니다. 로그 파일에서 `IMU1.MagX`로 나타납니다.

    # device ID 66826 in 24-bit binary:
    00000001  00000101  00001 010
    
    # decodes to:
    HMC5883   dev 5   bus 1 SPI
    

SPI로 연결한 내부 MPU9250 지자계센서입니다. 버스 1번에서 슬롯 4번을 선택한 하위노드입니다. 로그 파일에서 `IMU2.MagX`로 나타납니다.

    # device ID 263178 in 24-bit binary:
    00000100  00000100  00001 010
    
    #decodes to:
    MPU9250   dev 4   bus 1 SPI
    

### 장치 ID 인코딩

장치 ID는 다음 형식의 24비트 숫자입니다. 참고로 위 디코딩 예제에서 처음 필드는 최하위 비트입니다.

```C
struct DeviceStructure {
  enum DeviceBusType bus_type : 3;
  uint8_t bus: 5;    // which instance of the bus type
  uint8_t address;   // address on the bus (eg. I2C address)
  uint8_t devtype;   // device class specific device type
};
```

`bus_type`은 다음과 같이 디코딩하며:

```C
enum DeviceBusType {
  DeviceBusType_UNKNOWN = 0,
  DeviceBusType_I2C     = 1,
  DeviceBusType_SPI     = 2,
  DeviceBusType_UAVCAN  = 3,
};
```

`devtype`은 다음 내용대로 디코딩합니다:

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

## 디버깅

일반 디버깅 주제는 [디버깅/로깅](../debug/README.md)을 살펴보십시오.

### 상세 기록

드라이버 (와 기타 모듈) 에서는 기본적으로 최소한 자세한 로그 문자열을 출력합니다(예: `PX4_DEBUG`, `PX4_WARN`, `PX4_ERR` 등)

로그의 상세도는 `RELEASE_BUILD` (기본), `DEBUG_BUILD` (상세), `TRACE_BUILD` (매우 상세) 매크로로 빌드 시간에 정의할 수 있습니다. 

드라이버의 소스트리에 있는 CMakeLists.txt의 `px4_add_module` 함수에서 `COMPILE_FLAGS`로깅 수준 값을 바꾸십시오. 아래 코드 단편은 단일 모듈 또는 드라이버에서의 DEBUG_BUILD 레벨 디버깅을 활성화할 때 바꿀 필요가 있는 부분을 보여줍니다.

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
    

> **Tip** 상세 로깅은 .cpp 파일 최상단에 `#define DEBUG_BUILD` 행을 추가하여 파일 기반 별로 활성화할 수 있습니다.