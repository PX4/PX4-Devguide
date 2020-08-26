# 모듈 참고: 드라이버

하위 분류:

- [관성 센서](modules_driver_imu.md)
- [거리 센서](modules_driver_distance_sensor.md)
- [항속 센서](modules_driver_airspeed_sensor.md)
- [기압](modules_driver_baro.md)
- [자력계](modules_driver_magnetometer.md)

## adc

소스 코드: [drivers/adc](https://github.com/PX4/Firmware/tree/master/src/drivers/adc)

### 설명

ADC 드라이버입니다.

### 사용법 {#adc_usage}

    adc <command> [arguments...]
     Commands:
       start
    
       test
    
       stop
    
       status        print status info
    

## atxxxx

소스 코드: [drivers/osd/atxxxx](https://github.com/PX4/Firmware/tree/master/src/drivers/osd/atxxxx)

### 설명

OmnibusF4SD 보드에 붙은것과 같은 ATXXXX 칩용 OSD 드라이버입니다.

OSD_ATXXXX_CFG 매개변수로 활성화할 수 있습니다.

### 사용법 {#atxxxx_usage}

    atxxxx <command> [arguments...]
     Commands:
       start
         [-s]        Internal SPI bus(es)
         [-S]        External SPI bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-c <val>]  chip-select index (for external SPI)
                     default: 1
         [-m <val>]  SPI mode
         [-f <val>]  bus frequency in kHz
         [-q]        quiet startup (no message if no device found)
    
       stop
    
       status        print status info
    

## batt_smbus

소스 코드: [drivers/batt_smbus](https://github.com/PX4/Firmware/tree/master/src/drivers/batt_smbus)

### 설명

BQ40Z50 잔여량 측정 IC용 지능형 배터리 드라이버.

### 예제

다음 설정 매개변수를 플래시에 기록합니다: address, number_of_bytes, byte0, ... , byteN

    batt_smbus -X write_flash 19069 2 27 0
    

### 사용법 {#batt_smbus_usage}

    batt_smbus <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-q]        quiet startup (no message if no device found)
         [-a <val>]  I2C address
                     default: 11
    
       man_info      Prints manufacturer info.
    
       unseal        Unseals the devices flash memory to enable write_flash
                     commands.
    
       seal          Seals the devices flash memory to disbale write_flash commands.
    
       suspend       Suspends the driver from rescheduling the cycle.
    
       resume        Resumes the driver from suspension.
    
       write_flash   Writes to flash. The device must first be unsealed with the
                     unseal command.
         [address]   The address to start writing.
         [number of bytes] Number of bytes to send.
         [data[0]...data[n]] One byte of data at a time separated by spaces.
    
       stop
    
       status        print status info
    

## blinkm

소스 코드: [drivers/lights/blinkm](https://github.com/PX4/Firmware/tree/master/src/drivers/lights/blinkm)

### 사용법 {#blinkm_usage}

    blinkm <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-q]        quiet startup (no message if no device found)
         [-a <val>]  I2C address
                     default: 9
    
       systemstate
    
       ledoff
    
       list
    
       script
         -n <val>    Script file name
                     values: <file>
    
       stop
    
       status        print status info
    

## bst

소스 코드: [drivers/telemetry/bst](https://github.com/PX4/Firmware/tree/master/src/drivers/telemetry/bst)

### 사용법 {#bst_usage}

    bst <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-q]        quiet startup (no message if no device found)
         [-a <val>]  I2C address
                     default: 118
    
       stop
    
       status        print status info
    

## dshot

소스 코드: [drivers/dshot](https://github.com/PX4/Firmware/tree/master/src/drivers/dshot)

### 설명

DShot 출력 드라이버입니다. FMU 드라이버와 유사하나, PWM 대신 ESC 통신 프로토콜처럼 DShot용 대체 용도로 활용할 수 있습니다.

다음 항목, 기능을 지원합니다:

- DShot150, DShot300, DShot600, DShot1200
- 별도의 UART를 통한 텔레메트리 통신, esc_status 메세지 전송
- CLI로의 DShot 명령 전송

### 예제

모터 1번의 영구 역방향 설정:

    dshot reverse -m 1
    dshot save -m 1
    

저장 후, 기존 회전 방향과는 반대 방향으로 모터가 동작합니다. 동일한 명령을 반복하면 다시 회전 방향을 반전합니다.

### 사용법 {#dshot_usage}

    dshot <command> [arguments...]
     Commands:
       start         Start the task (without any mode set, use any of the mode_*
                     cmds)
    
     All of the mode_* commands will start the module if not running already
    
       mode_gpio
    
       mode_pwm      Select all available pins as PWM
    
       mode_pwm8
    
       mode_pwm6
    
       mode_pwm5
    
       mode_pwm5cap1
    
       mode_pwm4
    
       mode_pwm4cap1
    
       mode_pwm4cap2
    
       mode_pwm3
    
       mode_pwm3cap1
    
       mode_pwm2
    
       mode_pwm2cap2
    
       mode_pwm1
    
       telemetry     Enable Telemetry on a UART
         <device>    UART device
    
       reverse       Reverse motor direction
         [-m <val>]  Motor index (1-based, default=all)
    
       normal        Normal motor direction
         [-m <val>]  Motor index (1-based, default=all)
    
       save          Save current settings
         [-m <val>]  Motor index (1-based, default=all)
    
       3d_on         Enable 3D mode
         [-m <val>]  Motor index (1-based, default=all)
    
       3d_off        Disable 3D mode
         [-m <val>]  Motor index (1-based, default=all)
    
       beep1         Send Beep pattern 1
         [-m <val>]  Motor index (1-based, default=all)
    
       beep2         Send Beep pattern 2
         [-m <val>]  Motor index (1-based, default=all)
    
       beep3         Send Beep pattern 3
         [-m <val>]  Motor index (1-based, default=all)
    
       beep4         Send Beep pattern 4
         [-m <val>]  Motor index (1-based, default=all)
    
       beep5         Send Beep pattern 5
         [-m <val>]  Motor index (1-based, default=all)
    
       esc_info      Request ESC information
         -m <val>    Motor index (1-based)
    
       stop
    
       status        print status info
    

## fake_magnetometer

소스 코드: [examples/fake_magnetometer](https://github.com/PX4/Firmware/tree/master/src/examples/fake_magnetometer)

### 설명

가상 지자계(sensor_mag)로서 지자계데이터를 내보냅니다. vehicle_attitude와 vehicle_gps_position이 필요합니다.

### 사용법 {#fake_magnetometer_usage}

    fake_magnetometer <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## gps

소스 코드: [drivers/gps](https://github.com/PX4/Firmware/tree/master/src/drivers/gps)

### 설명

장치와의 통신을 처리하는 GPS 드라이버 모듈이며, uORB로 위치 정보를 내보냅니다. 여러 (장치 제조사의) 프로토콜을 지원하며, 기본적으로 해당 프로토콜을 자동으로 선택합니다.

모듈에서는 `-e` 매개변수로 지정하면 2차 GPS 장치를 지원합니다. 2차 uORB 토픽 인스턴스에서 위치 정보를 내보내나, 현재는 시스템 대부분에서 사용하지 않습니다(다만, 데이터는 기록하기 때문에, 비교 목적으로 활용할 수 있음).

### 구현

각 장치의 데이터 폴링용 스레드가 있습니다. GPS 프로토콜 클래스는 콜백 함수로 구현하므로 다른 프로젝트에서 마찬가지로 사용할 수 있습니다(예: QGroundControl에서도 사용).

### 예제

시험을 진행할 때 GPS 신호를 속이는 목적으로 쓸만합니다(유효한 위치 정보를 가지고 있다고 시스템에 시그널을 보냄):

    gps stop
    gps start -f
    

GPS 장치 2개를 시작하려면(주 GPS 장치는 /dev/ttyS3에, 보조 GPS 장치는 /dev/ttyS4):

    gps start -d /dev/ttyS3 -e /dev/ttyS4
    

전원 인가 상태에서 GPS 장치를 다시 시작하려면

    gps reset warm
    

### 사용법 {#gps_usage}

    gps <command> [arguments...]
     Commands:
       start
         [-d <val>]  GPS device
                     values: <file:dev>, default: /dev/ttyS3
         [-b <val>]  Baudrate (can also be p:<param_name>)
                     default: 0
         [-e <val>]  Optional secondary GPS device
                     values: <file:dev>
         [-g <val>]  Baudrate (secondary GPS, can also be p:<param_name>)
                     default: 0
         [-f]        Fake a GPS signal (useful for testing)
         [-s]        Enable publication of satellite info
         [-i <val>]  GPS interface
                     values: spi|uart, default: uart
         [-p <val>]  GPS Protocol (default=auto select)
                     values: ubx|mtk|ash|eml
    
       stop
    
       status        print status info
    
       reset         Reset GPS device
         cold|warm|hot Specify reset type
    

## ina226

소스 코드: [drivers/power_monitor/ina226](https://github.com/PX4/Firmware/tree/master/src/drivers/power_monitor/ina226)

### 설명

INA226 전력 감시 칩 드라이버입니다.

각 인스턴스에 개별 버스 또는 I2C 주소를 부여받았다면 이 드라이버의 다중 인스턴스를 동시에 실행할 수 있습니다.

예를 들어, 어떤 인스턴스는 2번 버스, 주소 0x41에서 동작할 수 있고, 다른 인스턴스는 2번 버스, 주소 0x43에서 동작할 수 있습니다.

INA226 모듈에 전원을 인가하지 않으면, 기본적으로 드라이버 초기화에 실패합니다. 이 문제를 수정하려면 -f 플래그를 사용하십시오. 이 플래그를 설정하고 나서도 초기화에 실패하면, 드라이버는 0.5초당 한번씩 초기화를 다시 시도합니다. 이 플래그를 설정하고 나면, 드라이버를 시작하고 난 후에도 배터리를 연결할 수 있고, 그 후에 이 드라이버가 동작합니다. 이 플래그를 설정하지 않으면, 드라이버를 시작하기 전에 배터리를 연결해두어야합니다.

### 사용법 {#ina226_usage}

    ina226 <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-q]        quiet startup (no message if no device found)
         [-a <val>]  I2C address
                     default: 65
         [-k]        if initialization (probing) fails, keep retrying periodically
         [-t <val>]  battery index for calibration values (1 or 2)
                     default: 1
    
       stop
    
       status        print status info
    

## irlock

소스 코드: [drivers/irlock](https://github.com/PX4/Firmware/tree/master/src/drivers/irlock)

### 사용법 {#irlock_usage}

    irlock <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-q]        quiet startup (no message if no device found)
         [-a <val>]  I2C address
                     default: 84
    
       stop
    
       status        print status info
    

## lsm303agr

소스 코드: [drivers/magnetometer/lsm303agr](https://github.com/PX4/Firmware/tree/master/src/drivers/magnetometer/lsm303agr)

### 사용법 {#lsm303agr_usage}

    lsm303agr <command> [arguments...]
     Commands:
       start
         [-s]        Internal SPI bus(es)
         [-S]        External SPI bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-c <val>]  chip-select index (for external SPI)
                     default: 1
         [-m <val>]  SPI mode
         [-f <val>]  bus frequency in kHz
         [-q]        quiet startup (no message if no device found)
         [-R <val>]  Rotation
                     default: 0
    
       stop
    
       status        print status info
    

## paw3902

소스 코드: [drivers/optical_flow/paw3902](https://github.com/PX4/Firmware/tree/master/src/drivers/optical_flow/paw3902)

### 사용법 {#paw3902_usage}

    paw3902 <command> [arguments...]
     Commands:
       start
         [-s]        Internal SPI bus(es)
         [-S]        External SPI bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-c <val>]  chip-select index (for external SPI)
                     default: 1
         [-m <val>]  SPI mode
         [-f <val>]  bus frequency in kHz
         [-q]        quiet startup (no message if no device found)
         [-R <val>]  Rotation
                     default: 0
    
       stop
    
       status        print status info
    

## pca9685

소스 코드: [drivers/pca9685](https://github.com/PX4/Firmware/tree/master/src/drivers/pca9685)

### 사용법 {#pca9685_usage}

    pca9685 <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-q]        quiet startup (no message if no device found)
    
       reset
    
       test          enter test mode
    
       stop
    
       status        print status info
    

## pca9685_pwm_out

소스 코드: [drivers/pca9685_pwm_out](https://github.com/PX4/Firmware/tree/master/src/drivers/pca9685_pwm_out)

### 설명

이 모듈은 PCA9685 칩으로 PWM 펄스를 생성합니다.

actuator_controls 토픽을 수신하고, PWM 출력을 혼합하여 기록합니다.

### 구현

ModuleBase과 OutputModuleInterface를 기반으로 구현했습니다. IIC 통신은 CDev::I2C에 기반합니다

### 예제

보통 다음 명령으로 시작합니다:

    pca9685_pwm_out start -a 64 -b 1
    

믹서 파일을 불러오려면 `mixer` 명령을 활용하십시오. `mixer load /dev/pca9685 ROMFS/px4fmu_common/mixers/quad_x.main.mix`

### 사용법 {#pca9685_pwm_out_usage}

    pca9685_pwm_out <command> [arguments...]
     Commands:
       start         Start the task
         [-a <val>]  device address on this bus
                     default: 64
         [-b <val>]  bus that pca9685 is connected to
                     default: 1
    
       stop
    
       status        print status info
    

## pcf8583

소스 코드: [drivers/rpm/pcf8583](https://github.com/PX4/Firmware/tree/master/src/drivers/rpm/pcf8583)

### 사용법 {#pcf8583_usage}

    pcf8583 <command> [arguments...]
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
    

## pmw3901

소스 코드: [drivers/optical_flow/pmw3901](https://github.com/PX4/Firmware/tree/master/src/drivers/optical_flow/pmw3901)

### 사용법 {#pmw3901_usage}

    pmw3901 <command> [arguments...]
     Commands:
       start
         [-s]        Internal SPI bus(es)
         [-S]        External SPI bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-c <val>]  chip-select index (for external SPI)
                     default: 1
         [-m <val>]  SPI mode
         [-f <val>]  bus frequency in kHz
         [-q]        quiet startup (no message if no device found)
         [-R <val>]  Rotation
                     default: 0
    
       stop
    
       status        print status info
    

## pwm_out

소스 코드: [drivers/pwm_out](https://github.com/PX4/Firmware/tree/master/src/drivers/pwm_out)

### 설명

이 모듈은 출력을 제어하고 입력 핀의 신호를 읽습니다. 개별 입출력 칩이 없는 보드에서는(예: 픽스레이서), 메인 채널을 활용합니다. 보드에 입출력 칩이 있다면(예: 픽스호크), AUX 채널을 활용하며, px4io 드라이버를 주요 입출력 드라이버로 활용합니다.

actuator_controls 토픽을 수신하고, PWM 출력을 혼합하여 기록합니다.

mode_* 명령으로 모듈을 설정합니다. 이 명령으로 어떤 처음 N개의 핀을 드라이버에 할당할지를 정의합니다. mode_pwm4를 사용한다면, 카메라 촬영 드라이버 또는 PWM 범위 검색 드라이버에서 핀 5, 6개를 사용할 수 있습니다. 대신 pwm_out은 캡처 모드 중 하나로 시작할 수 있으며, 드라이버는 ioctl을 호출하여 캡처 콜백 함수를 등록할 수 있습니다.

### 구현

기본적으로 모듈은 uORB actuator_controls 토픽의 함수 콜백을 통한 작업 큐에서 동작합니다.

### 예제

보통 다음 명령으로 시작합니다:

    pwm_out mode_pwm
    

이 명령으로 모든 가용핀을 제어합니다.

입력을 잡아(신호 레벨의 상승 하강 순간) 콘솔에 출력합니다. pwm_out 을 원하는 캡처 모드로 시작합니다:

    pwm_out mode_pwm3cap1
    

이 명령으로 네번째 핀의 입력 신호 캡처를 시작합니다. 그 다음 명령을 실행해보십시오:

    pwm_out test
    

다른 설정(PWM 속도, 레벨 등)을 진행하려면 `pwm` 명령을 사용하시고, 믹서 파일을 불러오려면 `mixer` 명령을 사용하십시오.

### 사용법 {#pwm_out_usage}

    pwm_out <command> [arguments...]
     Commands:
       start         Start the task (without any mode set, use any of the mode_*
                     cmds)
    
     All of the mode_* commands will start pwm_out if not running already
    
       mode_gpio
    
       mode_pwm      Select all available pins as PWM
    
       mode_pwm8
    
       mode_pwm6
    
       mode_pwm5
    
       mode_pwm5cap1
    
       mode_pwm4
    
       mode_pwm4cap1
    
       mode_pwm4cap2
    
       mode_pwm3
    
       mode_pwm3cap1
    
       mode_pwm2
    
       mode_pwm2cap2
    
       mode_pwm1
    
       sensor_reset  Do a sensor reset (SPI bus)
         [<ms>]      Delay time in ms between reset and re-enabling
    
       peripheral_reset Reset board peripherals
         [<ms>]      Delay time in ms between reset and re-enabling
    
       i2c           Configure I2C clock rate
         <bus_id> <rate> Specify the bus id (>=0) and rate in Hz
    
       test          Test inputs and outputs
    
       stop
    
       status        print status info
    

## pwm_out_sim

소스 코드: [drivers/pwm_out_sim](https://github.com/PX4/Firmware/tree/master/src/drivers/pwm_out_sim)

### 설명

PWM 출력 가상 재현 드라이버입니다.

`actuator_control` uORB 메세지를 취하고, 사전에 불러온 믹서로 혼합한 후, `actuator_output` uORB 토픽에 결과를 실어 내보내는 유일한 함수입니다.

SITL과 HITL을 활용합니다.

### 사용법 {#pwm_out_sim_usage}

    pwm_out_sim <command> [arguments...]
     Commands:
       start         Start the module
         [-m <val>]  Mode
                     values: hil|sim, default: sim
    
       stop
    
       status        print status info
    

## px4flow

소스 코드: [drivers/optical_flow/px4flow](https://github.com/PX4/Firmware/tree/master/src/drivers/optical_flow/px4flow)

### 사용법 {#px4flow_usage}

    px4flow <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-q]        quiet startup (no message if no device found)
         [-a <val>]  I2C address
                     default: 66
         [-R <val>]  Rotation (default=downwards)
                     default: 25
    
       stop
    
       status        print status info
    

## rc_input

소스 코드: [drivers/rc_input](https://github.com/PX4/Firmware/tree/master/src/drivers/rc_input)

### 설명

이 모듈은 RC 입력 을 해석하며, 처리방식을 자동으로 선택합니다. 지원하는 처리 방식은 다음과 같습니다:

- PPM
- SBUS
- DSM
- SUMD
- ST24
- TBS 크로스파이어 (CRSF)

### 사용법 {#rc_input_usage}

    rc_input <command> [arguments...]
     Commands:
       start
         [-d <val>]  RC device
                     values: <file:dev>, default: /dev/ttyS3
    
       bind          Send a DSM bind command (module must be running)
    
       stop
    
       status        print status info
    

## rgbled

소스 코드: [drivers/lights/rgbled_ncp5623c](https://github.com/PX4/Firmware/tree/master/src/drivers/lights/rgbled_ncp5623c)

### 사용법 {#rgbled_usage}

    rgbled <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-q]        quiet startup (no message if no device found)
         [-a <val>]  I2C address
                     default: 57
    
       stop
    
       status        print status info
    

## roboclaw

소스 코드: [drivers/roboclaw](https://github.com/PX4/Firmware/tree/master/src/drivers/roboclaw)

### 설명

이 드라이버는 UART로 [Roboclaw 모터 드라이버](http://downloads.basicmicro.com/docs/roboclaw_user_manual.pdf)와 통신합니다. 이 드라이버는 두가지 작업을 수행합니다:

- `actuator_controls_0` uORB 토픽을 기반으로 모터를 제어합니다.
- 바퀴 인코더를 읽고 `wheel_encoders` uORB 토픽에 생짜 데이터를 내보냅니다

이 드라이버를 사용하려면 Roboclaw를 패킷 직렬 처리 모드로 두어야 하며(연결 문서 참고), 문서에 언급한대로 비행 조종 장치의 UART 포트를 Roboclaw에 연결해야 합니다. 픽스호크 4에서는, `/dev/ttyS3`에 대응하는 `UART & I2C B` 포트를 사용하십시오.

### 구현

이 모듈의 메인 루프( `RoboClaw.cpp::task_main()`에 있음)에서는 두가지 작업을 수행합니다:

1. Roboclaw가 가동중인 경우 `actuator_controls_0` 메세지를 기록합니다
2. Roboclaw의 인코더 데이터를 주기적으로 읽습니다.

UART 지연 때문에, 이 드라이버에서는 모든 단일 `actuator_controls_0` 메세지를 Roboclaw에 직접 기록하지 않습니다. 대신, `RBCLW_WRITE_PER` 값에 따라 기록 속도에 제한을 둡니다.

시작시, 이 드라이버는 Roboclaw의 연결 여부 확인을 위해 상태를 읽으려 합니다. 이 과정이 실패하면, 드라이버는 바로 멈춥니다.

### 예제

이 드라이버를 시작하는 명령은 다음과 같습니다:

$ roboclaw start <device> <baud>

`<device>`은(는) UART 포트 이름입니다. 픽스호크 4에서는 `/dev/ttyS3`입니다. `<baud>`은(는) 초당 비트 전송율입니다.

사용할 수 있는 명령어는 다음과 같습니다:

- `$ roboclaw start <device> <baud>`
- `$ roboclaw status`
- `$ roboclaw stop`

### 사용법 {#roboclaw_usage}

    roboclaw <command> [arguments...]
     Commands:
    

## safety_button

소스 코드: [drivers/safety_button](https://github.com/PX4/Firmware/tree/master/src/drivers/safety_button)

### 설명

이 모듈은 안전 단추 동작을 담당합니다. 안전 단추를 세번 누르면 GCS 페어링 요청을 신속하게 실행합니다.

### 사용법 {#safety_button_usage}

    safety_button <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## tap_esc

소스 코드: [drivers/tap_esc](https://github.com/PX4/Firmware/tree/master/src/drivers/tap_esc)

### 설명

UART로 TAP_ESC 하드웨어를 제어하는 모듈입니다. actuator_controls 토픽을 수신하고, PWM 출력을 혼합하여 기록합니다.

### 구현

이 모듈은 스레드 버전으로만 구현했습니다. 이는 작업 큐에서 동작하는 대신, 자체 스레드에서 실행한다는 뜻입니다.

### 예제

모듈은 보통 다음 명령으로 시작합니다: tap_esc start -d /dev/ttyS2 -n <1-8>

### 사용법 {#tap_esc_usage}

    tap_esc <command> [arguments...]
     Commands:
       start         Start the task
         [-d <val>]  Device used to talk to ESCs
                     values: <device>
         [-n <val>]  Number of ESCs
                     default: 4
    

## vmount

소스 코드: [modules/vmount](https://github.com/PX4/Firmware/tree/master/src/modules/vmount)

### 설명

마운트 (짐벌) 제어 드라이버입니다. 각기 다른 입력 방식을(예: RC 또는 MAVLink) 설정 출력에(예: AUX 채널 또는 MAVLink) 대응합니다.

[gimbal_control](https://dev.px4.io/master/en/advanced/gimbal_control.html) 페이지에 사용법이 잘 나와있습니다.

### 구현

각 방식은 자체 클래스로 구현하며, 입출력용 공통 기반 클래스가 있습니다. `ControlData` 데이터 구조로 정의한 API로 연결합니다. 제각각의 입력 방식을 각 출력 방식에 대해 사용할 수 있음을 보여주며, 새 입출력 수단을 최소한의 노력으로 추가할 수 있습니다.

### 예제

고정 방위각을 설정한 출력 값을 시험합니다(그리고 다른 축은 0 값을 잡아줍니다):

    vmount stop
    vmount test yaw 30
    

### 사용법 {#vmount_usage}

    vmount <command> [arguments...]
     Commands:
       start
    
       test          Test the output: set a fixed angle for one axis (vmount must
                     not be running)
         roll|pitch|yaw <angle> Specify an axis and an angle in degrees
    
       stop
    
       status        print status info
    

## voxlpm

소스 코드: [drivers/power_monitor/voxlpm](https://github.com/PX4/Firmware/tree/master/src/drivers/power_monitor/voxlpm)

### 사용법 {#voxlpm_usage}

    voxlpm [arguments...]
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-q]        quiet startup (no message if no device found)
         [-T <val>]  Type
                     values: VBATT|P5VDC|P12VDC, default: VBATT
         [-K]        if initialization (probing) fails, keep retrying periodically
    
       stop
    
       status        print status info