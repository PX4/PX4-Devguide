# 모듈 참고: 거리 센서 (드라이버)

## leddar_one

소스 코드: [drivers/distance_sensor/leddar_one](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/leddar_one)

### 설명

LeddarOne LiDAR 직렬 버스 드라이버입니다.

대부분 보드는 SENS_LEDDAR1_CFG 매개변수로 지정 UART 에서 드라이버를 활성화/시작 하도록 설정했습니다.

설정/활용 정보: https://docs.px4.io/master/en/sensor/leddar_one.html

### 예제

지정 직렬 통신 장치에서 드라이버를 시작하려면

    leddar_one start -d /dev/ttyS1
    

드라이버 동작 중단

    leddar_one stop
    

### 사용법 {#leddar_one_usage}

    leddar_one <command> [arguments...]
     Commands:
       start         Start driver
         -d <val>    Serial device
         [-r <val>]  Sensor rotation - downward facing by default
                     default: 25
    
       stop          Stop driver
    

## ll40ls

소스 코드: [drivers/distance_sensor/ll40ls](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/ll40ls)

### 설명

LidarLite 범위 검색 센서용 I2C 버스 드라이버입니다.

센서/드라이버는 SENS_EN_LL40LS 매개변수를 활용하여 활성화해야합니다.

설정/활용 정보: https://docs.px4.io/master/en/sensor/lidar_lite.html

### 사용법 {#ll40ls_usage}

    ll40ls <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-q]        quiet startup (no message if no device found)
         [-R <val>]  Sensor rotation - downward facing by default
                     default: 25
    
       regdump
    
       stop
    
       status        print status info
    

## mappydot

소스 코드: [drivers/distance_sensor/mappydot](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/mappydot)

### 사용법 {#mappydot_usage}

    mappydot <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-q]        quiet startup (no message if no device found)
    
       stop
    
       status        print status info
    

## mb12xx

소스 코드: [drivers/distance_sensor/mb12xx](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/mb12xx)

### 사용법 {#mb12xx_usage}

    mb12xx <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-q]        quiet startup (no message if no device found)
         [-a <val>]  I2C address
                     default: 112
    
       set_address
         [-a <val>]  I2C address
                     default: 112
    
       stop
    
       status        print status info
    

## pga460

소스 코드: [drivers/distance_sensor/pga460](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/pga460)

### 설명

uORB로 장치와 통신하고 거리 정보를 내보내는 과정을 처리하는 초음파 범위 검색 드라이버입니다.

### 구현

이 드라이버는 NuttX 작업을 구현했습니다. 이 구현체는 work_queue 에서 지원하지 않는, UART를 통한 메세지 폴링에 필요하기 때문에 선택했습니다. 이 드라이버는 동작하는 동안 지속적으로 거리 측정을 진행합니다. 내보낼 데이터의 질을 개선하기 위해 거짓 정보를 탐지하는 간단한 알고리즘을 드라이버 수준에 구현했습니다. 드라이버는 센서 데이터가 안정적이지 않거나 잘못되었다고 판단하면 내보내지 않습니다.

### 사용법 {#pga460_usage}

    pga460 <command> [arguments...]
     Commands:
       start
         [device_path] The pga460 sensor device path, (e.g: /dev/ttyS6)
    
       status
    
       stop
    
       help
    

## sf0x

소스 코드: [drivers/distance_sensor/sf0x](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/sf0x)

### 설명

LightWare SF02/F, SF10/a, SF10/b, SF10/c, SF11/c 레이저 범위 검색 직렬 버스 통신 드라이버입니다.

대부분 보드는 SENS_SF0X_CFG 매개변수로 지정 UART 에서 드라이버를 활성화/시작 하도록 설정했습니다.

설정/활용 정보: https://docs.px4.io/master/en/sensor/sfxx_lidar.html

### 예제

지정 직렬 통신 장치에서 드라이버를 시작하려면

    sf0x start -d /dev/ttyS1
    

드라이버 동작 중단

    sf0x stop
    

### 사용법 {#sf0x_usage}

    sf0x <command> [arguments...]
     Commands:
       start         Start driver
         -d <val>    Serial device
         [-R <val>]  Sensor rotation - downward facing by default
                     default: 25
    
       stop          Stop driver
    

## sf1xx

소스 코드: [drivers/distance_sensor/sf1xx](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/sf1xx)

### 설명

Lightware SFxx 계열 SF10/a, SF10/b, SF10/c, SF11/c, SF/LW20 LIDAR 범위 검색 센서용 I2C 버스 드라이버입니다.

설정/활용 정보: https://docs.px4.io/master/en/sensor/sfxx_lidar.html

### 사용법 {#sf1xx_usage}

    sf1xx <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-q]        quiet startup (no message if no device found)
         [-R <val>]  Sensor rotation - downward facing by default
                     default: 25
    
       stop
    
       status        print status info
    

## srf02

소스 코드: [drivers/distance_sensor/srf02](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/srf02)

### 사용법 {#srf02_usage}

    srf02 <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-q]        quiet startup (no message if no device found)
         [-R <val>]  Sensor rotation - downward facing by default
                     default: 25
    
       stop
    
       status        print status info
    

## teraranger

소스 코드: [drivers/distance_sensor/teraranger](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/teraranger)

### 설명

TeraRanger 범위 검색 센서용 I2C 버스 드라이버입니다.

센서/드라이버는 SENS_EN_TRANGER 매개변수를 활용하여 활성화해야합니다.

설정/활용 정보: https://docs.px4.io/master/en/sensor/rangefinders.html#teraranger-rangefinders 

### 사용법 {#teraranger_usage}

    teraranger <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-q]        quiet startup (no message if no device found)
         [-R <val>]  Sensor rotation - downward facing by default
                     default: 25
    
       stop
    
       status        print status info
    

## tfmini

소스 코드: [drivers/distance_sensor/tfmini](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/tfmini)

### 설명

Benewake TFmini LiDAR용 직렬 버스 통신 드라이버입니다.

대부분 보드는 SENS_TFMINI_CFG 매개변수로 지정 UART 에서 드라이버를 활성화/시작 하도록 설정했습니다.

설정/활용 정보: https://docs.px4.io/master/en/sensor/tfmini.html

### 예제

지정 직렬 통신 장치에서 드라이버를 시작하려면

    tfmini start -d /dev/ttyS1
    

드라이버 동작 중단

    tfmini stop
    

### 사용법 {#tfmini_usage}

    tfmini <command> [arguments...]
     Commands:
       start         Start driver
         -d <val>    Serial device
         [-R <val>]  Sensor rotation - downward facing by default
                     default: 25
    
       status        Driver status
    
       stop          Stop driver
    
       test          Test driver (basic functional tests)
    
       status        Print driver status
    

## ulanding_radar

소스 코드: [drivers/distance_sensor/ulanding_radar](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/ulanding_radar)

### 설명

Aerotenna uLanding 레이더 장치용 직렬 버스 통신 드라이버입니다. 

설정/활용 정보: https://docs.px4.io/v1.9.0/en/sensor/ulanding_radar.html

### 예시

지정 직렬 통신 장치에서 드라이버를 시작하려면

    ulanding_radar start -d /dev/ttyS1
    

드라이버 동작 중단

    ulanding_radar stop
    

### 사용법 {#ulanding_radar_usage}

    ulanding_radar <command> [arguments...]
     Commands:
       start         Start driver
         -d <val>    Serial device
                     values: <file:dev>, default: /dev/ttyS3
         [-R <val>]  Sensor rotation - downward facing by default
                     default: 25
    
       stop          Stop driver
    

## vl53l0x

소스 코드: [drivers/distance_sensor/vl53l0x](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/vl53l0x)

### 사용법 {#vl53l0x_usage}

    vl53l0x <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-q]        quiet startup (no message if no device found)
         [-R <val>]  Sensor rotation - downward facing by default
                     default: 25
    
       stop
    
       status        print status info