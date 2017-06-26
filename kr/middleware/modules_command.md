# Modules Reference: Command
## bl_update
소스: [systemcmds/bl_update](https://github.com/PX4/Firmware/tree/master/src/systemcmds/bl_update)

파일에서 bootloader를 플래쉬하는 유틸리티

### 사용법
```
bl_update [arguments...]
   setopt        Set option bits to unlock the FLASH (only needed if in locked
                 state)

   <file>        Bootloader bin file
```
## 설정
소스: [systemcmds/config](https://github.com/PX4/Firmware/tree/master/src/systemcmds/config)

센서 드라이버 설정(샘플링 & public rate, range 등)
### 사용법
```
config <command> [arguments...]
 Commands:

 The <file:dev> argument is typically one of /dev/{gyro,accel,mag}i
   block         Block sensor topic publication
     <file:dev>  Sensor device file

   unblock       Unblock sensor topic publication
     <file:dev>  Sensor device file

   sampling      Set sensor sampling rate
     <file:dev> <rate> Sensor device file and sampling rate in Hz

   rate          Set sensor publication rate
     <file:dev> <rate> Sensor device file and publication rate in Hz

   range         Set sensor measurement range
     <file:dev> <rate> Sensor device file and range

   check         Perform sensor self-test (and print info)
     <file:dev>  Sensor device file
```
## dumpfile
소스: [systemcmds/dumpfile](https://github.com/PX4/Firmware/tree/master/src/systemcmds/dumpfile)

덤프 파일 유틸리티. binary 모드에서 stdout으로 파일 사이즈와 내용을 출력(LF를 CR LF로 바꾸지 말것)
### 사용법
```
dumpfile [arguments...]
     <file>      File to dump
```
## esc_calib
소스: [systemcmds/esc_calib](https://github.com/PX4/Firmware/tree/master/src/systemcmds/esc_calib)

ESC 칼리브레이션을 위한 도구

칼리브레이션 절차(명령 실행을 통한 가이드):
- 프로펠러 제거, ESC 전원끄기
- attitude 제어기 정지: mc_att_control stop, fw_att_control 정지
- safety가 꺼져있는지 확인
- 이 명령 실행

### 사용법
```
esc_calib [arguments...]
     [-d <val>]  Select PWM output device
                 values: <file:dev>, default: /dev/pwm_output0
     [-l <val>]  Low PWM value in us
                 default: 1000
     [-h <val>]  High PWM value in us
                 default: 2000
     [-c <val>]  select channels in the form: 1234 (1 digit per channel,
                 1=first)
     [-m <val>]  Select channels via bitmask (eg. 0xF, 3)
                 default: 0
     [-a]        Select all channels
```
## hardfault_log
소스: [systemcmds/hardfault_log](https://github.com/PX4/Firmware/tree/master/src/systemcmds/hardfault_log)

Hardfault 유틸리티

hardfaults를 처리하기 위해서 시작 스크립트에서 사용

### 사용법
```
hardfault_log <command> [arguments...]
 Commands:
   check         Check if there's an uncommited hardfault

   rearm         Drop an uncommited hardfault

   fault         Generate a hardfault (this command crashes the system :)
     [0|1]       Hardfault type: 0=divide by 0, 1=Assertion (default=0)

   commit        Write uncommited hardfault to /fs/microsd/fault_%i.txt (and
                 rearm, but don't reset)

   count         Read the reboot counter, counts the number of reboots of an
                 uncommited hardfault (returned as the exit code of the program)

   reset         Reset the reboot counter
```
## led_control
소스: [systemcmds/led_control](https://github.com/PX4/Firmware/tree/master/src/systemcmds/led_control)


### 설명
(외부) LED를 제어 및 테스트하기 위한 커맨드 라인 툴

이를 사용하기 위해서 실행되고 있는 드라이버가 있는지 확인. 이 드라이버가 led_control uorb topic을 처리.

여기에는 다양한 우선순위가 있는데 예를 들면 하나의 모듈이 낮은 우선순위로 색깔을 설정할 수 있고 다른 모듈은 높은 우선순위로 N번 깜빡이게 할 수 있습니다. 깜빡임 후에 LED는 자동으로 낮은 우선순위로 돌아갑니다. `reset` 명령은 더 낮은 우선순위로 사용할 수도 있습니다.

### 예제
처음 LED를 5회 파란색으로 깜빡임 :
```
led_control blink -c blue -l 0 -n 5
```


### 사용법
```
led_control <command> [arguments...]
 Commands:
   test          Run a test pattern

   on            Turn LED on

   off           Turn LED off

   reset         Reset LED priority

   blink         Blink LED N times
     [-n <val>]  Number of blinks
                 default: 3
     [-s <val>]  Set blinking speed
                 values: fast|normal|slow, default: normal

   breathe       Continuously fade LED in & out

 The following arguments apply to all of the above commands except for 'test':
     [-c <val>]  color
                 values: red|blue|green|yellow|purple|amber|cyan|white, default:
                 white
     [-l <val>]  Which LED to control: 0, 1, 2, ... (default=all)
                 default: -1
     [-p <val>]  Priority
                 default: 2
```
## listener
소스: [systemcmds/topic_listener](https://github.com/PX4/Firmware/tree/master/src/systemcmds/topic_listener)


uORB topic을 listen하는 유틸리티로 콘솔에 데이터를 출력합니다.

제약: topic의 처음 인스턴스만 listen할 수 있습니다.


### 사용법
```
listener [arguments...]
     <topic_name> [<num_msgs>] uORB topic name and optionally number of messages
                 (default=1)
```
## mixer
소스: [systemcmds/mixer](https://github.com/PX4/Firmware/tree/master/src/systemcmds/mixer)


### 설명
ESC driver로 믹서 파일을 로드하거나 추가합니다.

이 드라이버는 반드시 ioctl 사용을 지원해야만 합니다. NuttX에 해당되지만 RPi에는 해당되지 않습니다.

### 사용법
```
mixer <command> [arguments..Reference.]
 Commands:
   load
     <file:dev> <file> Output device (eg. /dev/pwm_output0) and mixer file

   append
     <file:dev> <file> Output device (eg. /dev/pwm_output0) and mixer file
```
## motor_ramp
소스: [systemcmds/motor_ramp](https://github.com/PX4/Firmware/tree/master/src/systemcmds/motor_ramp)


### 설명
모터를 ramp up 테스트를 위한 어플리케이션.

시작하기 전에, 실행 중인 attitude 제어기를 정지시켰는지 확인 :
```
mc_att_control stop
fw_att_control stop
```

시작할때, 백그라운드 태스크가 시작되고 실행하는데 수 초가 걸립니다(지정한 바와 같이). 그런 다음 빠져나옵니다.
When starting, a background task is started, runs for several seconds (as specified), then exits.

Note: 이 명령은 동시에 `/dev/pwm_output0` 출력만 지원합니다.

### 예제
```
motor_ramp sine 1100 0.5
```

### 사용법
```
motor_ramp [arguments...]
     ramp|sine|square mode
     <min_pwm> <time> [<max_pwm>] pwm value in us, time in sec

 WARNING: motors will ramp up to full speed!
```
## motor_test
소스: [systemcmds/motor_test](https://github.com/PX4/Firmware/tree/master/src/systemcmds/motor_test)

모터 테스트를 위한 유틸리티

Note: motor_test uorb topic을 지원하는 드라이버에 대해서만 사용할 수 있습니다.(현재 uavcan과 tap_esc)

### 사용법
```
motor_test <command> [arguments...]
 Commands:
   test          Set motor(s) to a specific output value
     [-m <val>]  Motor to test (0...7, all if not specified)
                 default: -1
     [-p <val>]  Power (0...100)
                 default: 0

   stop          Stop all motors

   iterate       Iterate all motors starting and stopping one after the other
```
## mtd
소스: [systemcmds/mtd](https://github.com/PX4/Firmware/tree/master/src/systemcmds/mtd)

파티션을 마운트하고 테스트하는 유틸리티(FRAM/EEPROM 스토리지 기반으로 해당 보드에서 정의)
### 사용법
```
mtd <command> [arguments...]
 Commands:
   status        Print status information

   start         Mount partitions

   readtest      Perform read test

   rwtest        Perform read-write test

   erase         Erase partition(s)

 The commands 'start', 'readtest', 'rwtest' and 'erase' have an optional
 parameter:
     [<partition_name1> [<partition_name2> ...]] Partition names (eg.
                 /fs/mtd_params), use system default if not provided
```
## nshterm
소스: [systemcmds/nshterm](https://github.com/PX4/Firmware/tree/master/src/systemcmds/nshterm)

NSH 쉘을 해당 포트에서 실행.

USB 시리얼 포트로 쉘을 시작하는데 사용했던 것입니다. 이제는 mavlink를 실행하며 mavlink 상에서 쉘을 사용하는 것이 가능합니다.

### 사용법
```
nshterm [arguments...]
     <file:dev>  Device on which to start the shell (eg. /dev/ttyACM0)
```
## param
소스: [systemcmds/param](https://github.com/PX4/Firmware/tree/master/src/systemcmds/param)


### 설명
쉘이나 스크립트로 파라미터를 접근하고 처리하는 명령.

airframe-specific 파라미터를 설정하는 시작 스크립트에서 예제로 사용합니다.

파라미터는 변경되면 자동으로 저장됩니다. 예로 `param set`. 이것들은 일반적으로 FRAM이나 SD 카드에 저장됩니다. `param select`은 차후 저장을 위한 저장 위치를 변경하는데 사용할 수 있습니다.(매번 부팅마다 (재)설정이 필요합니다)

각 파라미터는 'used' flag를 가지고 있고, 부팅동안 읽히는 경우 설정됩니다. 그라운드 컨트롤 스테이션과 관련 파라미터를만 보여주는데 사용합니다.

### 예제
airframe을 변경하고 airframe의 디폴트 파라미터가 로드되었는지 확인:
```
param set SYS_AUTOSTART 4001
param set SYS_AUTOCONFIG 1
reboot
```

### 사용법
```
param <command> [arguments...]
 Commands:
   load          Load params from a file (overwrite all)
     [<file>]    File name (use default if not given)

   import        Import params from a file
     [<file>]    File name (use default if not given)

   save          Save params to a file
     [<file>]    File name (use default if not given)

   select        Select default file
     [<file>]    File name (use <root>/eeprom/parameters if not given)

   show          Show parameter values
     [-c]        Show only changed params
     [<filter>]  Filter by param name (wildcard at end allowed, eg. sys_*)

   set           Set parameter to a value
     <param_name> <value> Parameter name and value to set
     [fail]      If provided, let the command fail if param is not found

   compare       Compare a param with a value. Command will succeed if equal
     <param_name> <value> Parameter name and value to compare

   greater       Compare a param with a value. Command will succeed if param is
                 greater than the value
     <param_name> <value> Parameter name and value to compare

   reset         Reset params to default
     [<exclude1> [<exclude2>]] Do not reset matching params (wildcard at end
                 allowed)

   reset_nostart Reset params to default, but keep SYS_AUTOSTART and
                 SYS_AUTOCONFIG
     [<exclude1> [<exclude2>]] Do not reset matching params (wildcard at end
                 allowed)

   index         Show param for a given index
     <index>     Index: an integer >= 0

   index_used    Show used param for a given index
     <index>     Index: an integer >= 0

   find          Show index of a param
     <param>     param name
```
## perf
소스: [systemcmds/perf](https://github.com/PX4/Firmware/tree/master/src/systemcmds/perf)

성능 카운터를 출력하는 도구
### 사용법
```
perf [arguments...]
   reset         Reset all counters

   latency       Print HRT timer latency histogram

 Prints all performance counters if no arguments given
```
## pwm
소스: [systemcmds/pwm](https://github.com/PX4/Firmware/tree/master/src/systemcmds/pwm)


### 설명
이 명령은 servo와 ESC 제어를 위한 PWM 출력을 설정하는데 사용합니다.

디폴트 장치 `/dev/pwm_output0`가 메인 채널이고 AUX 채널은 `/dev/pwm_output1`에 있습니다.(`-d` parameter).

PWM 파라미터 (`PWM_*`)를 적용하기 위해서 스타트업 스크립트에서 사용됩니다.(혹은 특정 airframe config에서 제공) `pwm info`는 현재 셋팅을 보여줍니다.(trim 값은 offset이고 `PWM_MAIN_TRIMx`와 `PWM_AUX_TRIMx`로 설정)

disarmed 값은 모터가 회전하지 않는 값으로 설정해야만 하며(이는 kill 스위치로 사용할 수도 있음), 회전해야하는 최소값.
채널들을 하나의 그룹에 할당합니다. 하드웨어 제약 때문에, 업데이트 rate는 그룹에 한해서만 설정할 수 있습니다. `pwm info`를 사용해서 해당 그룹을 출력할 수 있습니다. `-c` 인자를 사용하면, 포함된 그룹의 모든 채널은 반드시 포함해야만 합니다.

파라미터 `-p`와 `-r`은 정수를 지정하는 대신에 파라미터로 지정할 수 있습니다: 예제로 -p p:PWM_MIN을 사용합니다.

OneShot 모드에서 PWM 범위 [1000, 2000]는 자동으로 [125, 250]로 매핑됩니다.

### 예제
모든 채널을 400 Hz로 PWM rate을 설정:
```
pwm rate -a -r 400
```
채널 1과 3의 출력을 테스트하고 PWM 값을 1200 us로 설정:
```
pwm arm
pwm test -c 13 -p 1200
```


### 사용법
```
pwm <command> [arguments...]
 Commands:
   arm           Arm output

   disarm        Disarm output

   info          Print current configuration of all channels

   forcefail     Force Failsafe mode
     on|off      Turn on or off

   terminatefail Force Termination Failsafe mode
     on|off      Turn on or off

   rate          Configure PWM rates
     -r <val>    PWM Rate in Hz (0 = Oneshot, otherwise 50 to 400Hz)

   oneshot       Configure Oneshot125 (rate is set to 0)

   failsafe      Set Failsafe PWM value

   disarmed      Set Disarmed PWM value

   min           Set Minimum PWM value

   max           Set Maximum PWM value

   test          Set Output to a specific value until 'q' or 'c' or 'ctrl-c'
                 pressed

   steps         Run 5 steps from 0 to 100%

 The commands 'failsafe', 'disarmed', 'min', 'max' and 'test' require a PWM
 value:
     -p <val>    PWM value (eg. 1100)

 The commands 'rate', 'oneshot', 'failsafe', 'disarmed', 'min', 'max', 'test'
 and 'steps' additionally require to specify the channels with one of the
 following commands:
     [-c <val>]  select channels in the form: 1234 (1 digit per channel,
                 1=first)
     [-m <val>]  Select channels via bitmask (eg. 0xF, 3)
                 default: 0
     [-g <val>]  Select channels by group (eg. 0, 1, 2. use 'pwm info' to show
                 groups)
                 default: 0
     [-a]        Select all channels

 These parameters apply to all commands:
     [-d <val>]  Select PWM output device
                 values: <file:dev>, default: /dev/pwm_output0
     [-v]        Verbose output
     [-e]        Exit with 1 instead of 0 on error
```
## reboot
소스: [systemcmds/reboot](https://github.com/PX4/Firmware/tree/master/src/systemcmds/reboot)

시스템 리부팅
### Usage
```
reboot [arguments...]
     [-b]        Reboot into bootloader
```
## sd_bench
소스: [systemcmds/sd_bench](https://github.com/PX4/Firmware/tree/master/src/systemcmds/sd_bench)

SD 카드의 속도를 테스트
### 사용법
```
sd_bench [arguments...]
     [-b <val>]  Block size for each read/write
                 default: 4096
     [-r <val>]  Number of runs
                 default: 5
     [-d <val>]  Duration of a run in ms
                 default: 2000
     [-s]        Call fsync after each block (default=at end of each run)
```
## top
소스: [systemcmds/top](https://github.com/PX4/Firmware/tree/master/src/systemcmds/top)

실행 중인 프로세스, CPU, 스택 사용, 우선순위와 상태를 모니터링
### Usage
```
top [arguments...]
   once          print load only once
```
## usb_connected
소스: [systemcmds/usb_connected](https://github.com/PX4/Firmware/tree/master/src/systemcmds/usb_connected)

USB가 연결되어 있는지 확인하는 유틸리티. 이전에는 시작 스크립트에서 사용했었음. 리턴값 0은 USB가 연결되어 있다는 뜻이고 그렇지 않은 경우 1.
### 사용법
```
usb_connected [arguments...]
```
## ver
소스: [systemcmds/ver](https://github.com/PX4/Firmware/tree/master/src/systemcmds/ver)

여러 버전 정보를 출력하는 도구
### 사용법
```
ver <command> [arguments...]
 Commands:
   hw            Hardware architecture

   mcu           MCU info

   git           git version information

   bdate         Build date and time

   gcc           Compiler info

   bdate         Build date and time

   uid           UUID

   mfguid        Manufacturer UUID

   uri           Build URI

   all           Print all versions

   hwcmp         Compare hardware version (returns 0 on match)
     <hw>        Hardware to compare against (eg. PX4FMU_V4)
```
