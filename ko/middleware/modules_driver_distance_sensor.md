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
    

## lightware_laser_i2c

Source: [drivers/distance_sensor/lightware_laser_i2c](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/lightware_laser_i2c)

### 설명

I2C bus driver for Lightware SFxx series LIDAR rangefinders: SF10/a, SF10/b, SF10/c, SF11/c, SF/LW20.

Setup/usage information: https://docs.px4.io/master/en/sensor/sfxx_lidar.html

### Usage {#lightware_laser_i2c_usage}

    lightware_laser_i2c <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-q]        quiet startup (no message if no device found)
         [-R <val>]  Sensor rotation - downward facing by default
                     default: 25
    
       stop
    
       status        print status info
    

## lightware_laser_serial

Source: [drivers/distance_sensor/lightware_laser_serial](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/lightware_laser_serial)

### Description

Serial bus driver for the LightWare SF02/F, SF10/a, SF10/b, SF10/c, SF11/c Laser rangefinders.

Most boards are configured to enable/start the driver on a specified UART using the SENS_SF0X_CFG parameter.

Setup/usage information: https://docs.px4.io/master/en/sensor/sfxx_lidar.html

### Examples

Attempt to start driver on a specified serial device.

    lightware_laser_serial start -d /dev/ttyS1
    

Stop driver

    lightware_laser_serial stop
    

### Usage {#lightware_laser_serial_usage}

    lightware_laser_serial <command> [arguments...]
     Commands:
       start         Start driver
         -d <val>    Serial device
         [-R <val>]  Sensor rotation - downward facing by default
                     default: 25
    
       stop          Stop driver
    

## ll40ls

Source: [drivers/distance_sensor/ll40ls_pwm](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/ll40ls_pwm)

### Description

PWM driver for LidarLite rangefinders.

The sensor/driver must be enabled using the parameter SENS_EN_LL40LS.

Setup/usage information: https://docs.px4.io/master/en/sensor/lidar_lite.html

### Usage {#ll40ls_usage}

    ll40ls <command> [arguments...]
     Commands:
       start         Start driver
         [-R <val>]  Sensor rotation - downward facing by default
                     default: 25
    
       status        Print driver status information
    
       stop          Stop driver
    

## mappydot

Source: [drivers/distance_sensor/mappydot](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/mappydot)

### Usage {#mappydot_usage}

    mappydot <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-q]        quiet startup (no message if no device found)
    
       stop
    
       status        print status info
    

## mb12xx

Source: [drivers/distance_sensor/mb12xx](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/mb12xx)

### Usage {#mb12xx_usage}

    mb12xx <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
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

Source: [drivers/distance_sensor/pga460](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/pga460)

### Description

Ultrasonic range finder driver that handles the communication with the device and publishes the distance via uORB.

### Implementation

This driver is implented as a NuttX task. This Implementation was chosen due to the need for polling on a message via UART, which is not supported in the work_queue. This driver continuously takes range measurements while it is running. A simple algorithm to detect false readings is implemented at the driver levelin an attemptto improve the quality of data that is being published. The driver will not publish data at all if it deems the sensor data to be invalid or unstable.

### Usage {#pga460_usage}

    pga460 <command> [arguments...]
     Commands:
       start
         [device_path] The pga460 sensor device path, (e.g: /dev/ttyS6)
    
       status
    
       stop
    
       help
    

## srf02

소스 코드: [drivers/distance_sensor/srf02](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/srf02)

### 사용법 {#srf02_usage}

    srf02 <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
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
         [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
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
         [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-q]        quiet startup (no message if no device found)
         [-R <val>]  Sensor rotation - downward facing by default
                     default: 25
    
       stop
    
       status        print status info