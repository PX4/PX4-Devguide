# Modules 레퍼런스: Driver
## fmu
소스: [drivers/px4fmu](https://github.com/PX4/Firmware/tree/master/src/drivers/px4fmu)


### 설명
이 모듈은 출력을 유도하고 입력 핀을 읽는 역할을 담당합니다. 독립된 IO 칩 없이 보드에(Pixracer) 대해서 메인 채널을 사용합니다. IO 칩을 가진 보드(Pixhawk)에서는 AUX 채널을 사용하며 px4io driver가 메인으로 사용됩니다.

actuator_controls topic을 listen하고 믹싱과 PWM 출력을 내는 일을 합니다.
추가로 RC 입력을 파싱하고 자동으로 방식을 선택합니다. 지원하는 방식은 :
- PPM
- SBUS
- DSM
- SUMD
- ST24

mode_* commands를 통해서 설정합니다. 드라이버가 사용할 처음 N개 핀이 어떤 것인지를 정의합니다.
예제로 mode_pwm4을 사용함으로 pins 5와 6은 카메라 트리거 드라이버sk PWM rangefinder 드라이버에서 사용할 수 있습니다. 선택적으로 fmu는 캡쳐 모드 중에 하나로 시작시킬 수 있으며 드라이버는 ioctl 호출로 캡쳐 콜백을 등록할 수 있습니다.

### 구현
기본적으로 모듈은 work queue에서 실행되어 RAM 사용을 줄입니다. 자신의 thread에서 실행될 수도 있습니다. 이경우 시작 flag를 -t 로 지정해야하며 지연시간을 줄이게 됩니다. work queue에서 실행되는 경우, 고정 frequency로 스캐쥴되며 pwm rate는  actuator_controls topics의 갱신 rate를 제한합니다. 자신의 thread에서 실행되는 경우, 해당 모듈은 actuator_controls topic을 poll합니다.
추가로 pwm rate는 lower-level IO 타이머 rate를 정의합니다.

### 예제
일반적으로 다음과 같이 구동시킵니다:
```
fmu mode_pwm
```
유효한 모든 pin을 유도.

입력(rising와 falling edges)을 캡쳐하고 콘솔에 출력: 캡쳐 모드 중에 하나로 fmu 구동시키기:
```
fmu mode_pwm3cap1
```
이렇게 하면 4번째 핀의 캡쳐를 활성화 시킴. 다음으로 :
```
fmu test
```
추가 설정(PWM rate, levels, ...)을 위해서 `pwm` 명령을 사용하며 믹서 파일을 로드하기 위해서 `mixer` 명령을 사용합니다.

### 사용법
```
fmu <command> [arguments...]
 Commands:
   start         Start the task (without any mode set, use any of the mode_*
                 cmds)
     [-t]        Run as separate task instead of the work queue

 All of the mode_* commands will start the fmu if not running already

   mode_gpio

   mode_rcin     Only do RC input, no PWM outputs

   mode_pwm      Select all available pins as PWM

   mode_pwm1

   mode_pwm4

   mode_pwm2

   mode_pwm3

   mode_pwm3cap1

   mode_pwm2cap2

   mode_serial

   mode_gpio_serial

   mode_pwm_serial

   mode_pwm_gpio

   bind          Send a DSM bind command (module must be running)

   sensor_reset  Do a sensor reset (SPI bus)
     [<ms>]      Delay time in ms between reset and re-enabling

   peripheral_reset Reset board peripherals
     [<ms>]      Delay time in ms between reset and re-enabling

   i2c           Configure I2C clock rate
     <bus_id> <rate> Specify the bus id (>=0) and rate in Hz

   test          Test inputs and outputs

   fake          Arm and send an actuator controls command
     <roll> <pitch> <yaw> <thrust> Control values in range [-100, 100]

   stop

   status        print status info
```
## gps
소스: [drivers/gps](https://github.com/PX4/Firmware/tree/master/src/drivers/gps)


### 설명
GPS 드라이버 모듈은 장치와 통신을 처리하고 uORB를 통해 위치를 publish합니다. 다양한 프로토콜(장치 벤더)을 지원하며 기본적으로 자동으로 알맞는 프로토콜을 선택합니다.

이 모듈은 2번째 추가 GPS 장치를 지원하며 `-e` 파라미터로 지정합니다. 위치는 두번째 uORB topic 인스턴스로 publish되며 현재는 시스템의 다른 부분에서 사용하지는 않습니다.(하지만 데이터는 로깅되므로 비교하는데 사용할 수 있음)

### 구현
데이터를 각 장치가 polling하기 위한 thread가 있습니다. GPS 프로토콜 클래스는 콜백으로 구현되어 있어서 다른 프로젝트에서도 사용이 가능합니다.(예로 QGroundControl도 이를 사용)

### 예제
가짜 GPS 신호는 테스팅에 유용합니다. (시스템에 유효한 위치정보를 가지고 있다고 알림):
```
gps stop
gps start -f
```

### 사용법
```
gps <command> [arguments...]
 Commands:
   start
     [-d <val>]  GPS device
                 values: <file:dev>, default: /dev/ttyS3
     [-e <val>]  Optional secondary GPS device
                 values: <file:dev>
     [-f]        Fake a GPS signal (useful for testing)
     [-s]        Enable publication of satellite info
     [-i <val>]  GPS interface
                 values: spi|uart, default: uart
     [-p <val>]  GPS Protocol (default=auto select)
                 values: ubx|mtk|ash

   stop

   status        print status info
```
## vmount
소스: [drivers/vmount](https://github.com/PX4/Firmware/tree/master/src/drivers/vmount)


### 설명
마운트(짐벌) 제어 드라이버. 여러 다른 입력 방식을(예로 RC나 MAVLink) 설정한 출력에(예로 AUX 채널이나 MAVLink) 매핑합니다.

[gimbal_control](https://dev.px4.io/en/advanced/gimbal_control.html) 문서에서는 사용하는 방법을 소개합니다.

### 구현
각 방식은 자기 클래스에 구현되어 있습니다. 입력과 출력에 대한 공용 base 클래스가 있습니다. API를 통해 연결되고 `ControlData` 자료 구조에서 정의됩니다. 각 입력 방식은 각 출력 방식과 사용할 수 있습니다. 그리고 새로운 입력/출력은 적은 노력으로 추가할 수 있습니다.

### 예제
고정 yaw angle을 설정해서 출력을 테스트합니다.(그리고 다른 축은 0으로):
```
vmount stop
vmount test yaw 30
```

### 사용법
```
vmount <command> [arguments...]
 Commands:
   start

   test          Test the output: set a fixed angle for one axis (vmount must
                 not be running)
     roll|pitch|yaw <angle> Specify an axis and an angle in degrees

   stop

   status        print status info
```
