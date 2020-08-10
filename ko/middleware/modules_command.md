# 모듈 참고서: 명령어

## bl_update

원본: [systemcmds/bl_update](https://github.com/PX4/Firmware/tree/master/src/systemcmds/bl_update)

파일에서 부트로터를 플래싱하는 유틸리티

### 사용법 {#bl_update_usage}

    bl_update [arguments...]
       setopt        Set option bits to unlock the FLASH (only needed if in locked
                     state)
    
       <file>        Bootloader bin file
    

## dumpfile

원본: [systemcmds/dumpfile](https://github.com/PX4/Firmware/tree/master/src/systemcmds/dumpfile)

덤프 파일 유틸리티. 파일 크기와 내용을 표준 출력(터미널 창)에 바이너리 모드(LF를 CR LF로 바꾸지 않음)로 출력합니다.

### 사용법 {#dumpfile_usage}

    dumpfile [arguments...]
         <file>      File to dump
    

## dyn

원본: [systemcmds/dyn](https://github.com/PX4/Firmware/tree/master/src/systemcmds/dyn)

### 설명

PX4 바이너리로 컴파일하지 않은 동적 PX4 모듈을 불러오고 실행합니다.

### 예시

    dyn ./hello.px4mod start
    

### 사용법 {#dyn_usage}

    dyn [arguments...]
         <file>      File containing the module
         [arguments...] Arguments to the module
    

## esc_calib

원본: [systemcmds/esc_calib](https://github.com/PX4/Firmware/tree/master/src/systemcmds/esc_calib)

ESC 보정 도구

보정 과정(명령을 실행하면 인터페이스 메시지로 안내해줌):

- ESC의 프롭을 제거하고 전원을 끄십시오
- 고도, 속도 컨트롤러의 동작을 중단하십시오: mc_rate_control stop, fw_att_control stop
- safety가 off인지 확인하십시오
- 이 명령어를 실행하십시오

### 사용법 {#esc_calib_usage}

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
         [-a]        Select all channels
    

## gpio

원본: [systemcmds/gpio](https://github.com/PX4/Firmware/tree/master/src/systemcmds/gpio)

이 명령은 GPIO 시그널 읽기/쓰기에 활용합니다.

### 사용법 {#gpio_usage}

    gpio [arguments...]
       read
         <PORT> <PIN> GPIO port and pin
         [PULLDOWN|PULLUP] Pulldown/Pullup
         [--force]   Force (ignore board gpio list)
    
       write
         <PORT> <PIN> GPIO port and pin
         <VALUE>     Value to write
         [PULLDOWN|PULLUP] Pulldown/Pullup
         [--force]   Force (ignore board gpio list)
    

## hardfault_log

원본: [systemcmds/hardfault_log](https://github.com/PX4/Firmware/tree/master/src/systemcmds/hardfault_log)

하드웨어 문제를 다루는 유틸리티

하드웨어 문제를 처리하는 시작 스크립트에 활용합니다

### 사용법 {#hardfault_log_usage}

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
    

## i2cdetect

원본: [systemcmds/i2cdetect](https://github.com/PX4/Firmware/tree/master/src/systemcmds/i2cdetect)

각 버스에 연결한 I2C 장치를 검색하는 유틸리티

### 사용법 {#i2cdetect_usage}

    i2cdetect [arguments...]
         [-b <val>]  I2C bus
                     default: 1
    

## led_control

원본: [systemcmds/led_control](https://github.com/PX4/Firmware/tree/master/src/systemcmds/led_control)

### 설명

Command-line tool to control & test the (external) LED's.

To use it make sure there's a driver running, which handles the led_control uorb topic.

There are different priorities, such that for example one module can set a color with low priority, and another module can blink N times with high priority, and the LED's automatically return to the lower priority state after the blinking. The `reset` command can also be used to return to a lower priority.

### 예시

첫번째 LED를 파란색으로 5번 깜빡이려면:

    led_control blink -c blue -l 0 -n 5
    

### 사용법 {#led_control_usage}

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
    
       flash         Two fast blinks and then off with frequency of 1Hz
    
     The following arguments apply to all of the above commands except for 'test':
         [-c <val>]  color
                     values: red|blue|green|yellow|purple|amber|cyan|white, default:
                     white
         [-l <val>]  Which LED to control: 0, 1, 2, ... (default=all)
         [-p <val>]  Priority
                     default: 2
    

## listener

원본: [systemcmds/topic_listener](https://github.com/PX4/Firmware/tree/master/src/systemcmds/topic_listener)

uORB 요청을 수신하고 콘솔에 데이터를 출력하는 유틸리티

listener는 언제든지 Ctrl+C, Esc, Q를 입력하면 끝낼 수 있습니다.

### 사용법 {#listener_usage}

    listener <command> [arguments...]
     Commands:
         <topic_name> uORB topic name
         [-i <val>]  Topic instance
                     default: 0
         [-n <val>]  Number of messages
                     default: 1
         [-r <val>]  Subscription rate (unlimited if 0)
                     default: 0
    

## mixer

원본: [systemcmds/mixer](https://github.com/PX4/Firmware/tree/master/src/systemcmds/mixer)

### 설명

ESC 드라이버에 mixer 파일을 불러오거나 추가합니다.

참고로 라즈베리 파이가 아닌 NuttX의 경우, 드라이버에서 ioctl을 지원해야 합니다.

### 사용법 {#mixer_usage}

    mixer <command> [arguments...]
     Commands:
       load
         <file:dev> <file> Output device (eg. /dev/pwm_output0) and mixer file
    
       append
         <file:dev> <file> Output device (eg. /dev/pwm_output0) and mixer file
    

## motor_ramp

원본: [systemcmds/motor_ramp](https://github.com/PX4/Firmware/tree/master/src/systemcmds/motor_ramp)

### 설명

모터 출력 증가 테스트 프로그램입니다.

시작하기 전, 동작 중이던 모든 고도 컨트롤러를 멈추었는지 확인하십시오:

    mc_rate_control stop
    fw_att_control stop
    

프로그램 시작시, 백그라운드 작업을 시작하고 (지정한) 몇 초간 실행한 후 빠져나갑니다.

### 예시

    motor_ramp sine -a 1100 -r 0.5
    

### 사용법 {#motor_ramp_usage}

    motor_ramp [arguments...]
         ramp|sine|square mode
         [-d <val>]  Pwm output device
                     default: /dev/pwm_output0
         -a <val>    Select minimum pwm duty cycle in usec
         [-b <val>]  Select maximum pwm duty cycle in usec
                     default: 2000
         [-r <val>]  Select motor ramp duration in sec
                     default: 1.0
    
     WARNING: motors will ramp up to full speed!
    

## motor_test

원본: [systemcmds/motor_test](https://github.com/PX4/Firmware/tree/master/src/systemcmds/motor_test)

모터 테스트 유틸리티.

이 명령을 사용하기 전 모든 프로펠러를 제거하십시오.

### 사용법 {#motor_test_usage}

    motor_test <command> [arguments...]
     Commands:
       test          Set motor(s) to a specific output value
         [-m <val>]  Motor to test (0...7, all if not specified)
         [-p <val>]  Power (0...100)
                     default: 0
         [-t <val>]  Timeout in seconds (default=no timeout)
                     default: 0
         [-i <val>]  driver instance
                     default: 0
    
       stop          Stop all motors
    
       iterate       Iterate all motors starting and stopping one after the other
    

## mtd

원본: [systemcmds/mtd](https://github.com/PX4/Firmware/tree/master/src/systemcmds/mtd)

Utility to mount and test partitions (based on FRAM/EEPROM storage as defined by the board)

### 사용법 {#mtd_usage}

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
    

## nshterm

원본: [systemcmds/nshterm](https://github.com/PX4/Firmware/tree/master/src/systemcmds/nshterm)

설정 포트에서 NSH 셸을 시작합니다.

This was previously used to start a shell on the USB serial port. Now there runs mavlink, and it is possible to use a shell over mavlink.

### 사용법 {#nshterm_usage}

    nshterm [<인자값>...]
         <file:dev>  셸을 시작할 장치 (예: /dev/ttyACM0)
    

## param

원본: [systemcmds/param](https://github.com/PX4/Firmware/tree/master/src/systemcmds/param)

### 설명

Command to access and manipulate parameters via shell or script.

This is used for example in the startup script to set airframe-specific parameters.

Parameters are automatically saved when changed, eg. with `param set`. They are typically stored to FRAM or to the SD card. `param select` can be used to change the storage location for subsequent saves (this will need to be (re-)configured on every boot).

If the FLASH-based backend is enabled (which is done at compile time, e.g. for the Intel Aero or Omnibus), `param select` has no effect and the default is always the FLASH backend. However `param save/load <file>` can still be used to write to/read from files.

Each parameter has a 'used' flag, which is set when it's read during boot. It is used to only show relevant parameters to a ground control station.

### 예시

Change the airframe and make sure the airframe's default parameters are loaded:

    param set SYS_AUTOSTART 4001
    param set SYS_AUTOCONFIG 1
    reboot
    

### 사용법 {#param_usage}

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
         [-a]        Show all parameters (not just used)
         [-c]        Show only changed params (unused too)
         [-q]        quiet mode, print only param value (name needs to be exact)
         [<filter>]  Filter by param name (wildcard at end allowed, eg. sys_*)
    
       status        Print status of parameter system
    
       set           Set parameter to a value
         <param_name> <value> Parameter name and value to set
         [fail]      If provided, let the command fail if param is not found
    
       compare       Compare a param with a value. Command will succeed if equal
         [-s]        If provided, silent errors if parameter doesn't exists
         <param_name> <value> Parameter name and value to compare
    
       greater       Compare a param with a value. Command will succeed if param is
                     greater than the value
         [-s]        If provided, silent errors if parameter doesn't exists
         <param_name> <value> Parameter name and value to compare
         <param_name> <value> Parameter name and value to compare
    
       touch         Mark a parameter as used
         [<param_name1> [<param_name2>]] Parameter name (one or more)
    
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
    

## perf

원본: [systemcmds/perf](https://github.com/PX4/Firmware/tree/master/src/systemcmds/perf)

성능 카운터 출력 도구

### 사용법 {#perf_usage}

    perf [arguments...]
       reset         Reset all counters
    
       latency       Print HRT timer latency histogram
    
     어떤 인자도 부여하지 않았다면 모든 성능 카운터를 출력합니다
    

## pwm

원본: [systemcmds/pwm](https://github.com/PX4/Firmware/tree/master/src/systemcmds/pwm)

### 설명

This command is used to configure PWM outputs for servo and ESC control.

The default device `/dev/pwm_output0` are the Main channels, AUX channels are on `/dev/pwm_output1` (`-d` parameter).

It is used in the startup script to make sure the PWM parameters (`PWM_*`) are applied (or the ones provided by the airframe config if specified). `pwm info` shows the current settings (the trim value is an offset and configured with `PWM_MAIN_TRIMx` and `PWM_AUX_TRIMx`).

The disarmed value should be set such that the motors don't spin (it's also used for the kill switch), at the minimum value they should spin.

Channels are assigned to a group. Due to hardware limitations, the update rate can only be set per group. Use `pwm info` to display the groups. If the `-c` argument is used, all channels of any included group must be included.

The parameters `-p` and `-r` can be set to a parameter instead of specifying an integer: use -p p:PWM_MIN for example.

Note that in OneShot mode, the PWM range [1000, 2000] is automatically mapped to [125, 250].

### 예시

모든 채널의 PWM 속도를 400Hz로 설정하려면:

    pwm rate -a -r 400
    

예를 들어, 채널 1, 3번의 PWM 값을 1200us로 설정한다면:

    pwm arm
    pwm test -c 13 -p 1200
    

### 사용법 {#pwm_usage}

    pwm <command> [arguments...]
     Commands:
       arm           Arm output
    
       disarm        Disarm output
    
       info          Print current configuration of all channels
    
       forcefail     Force Failsafe mode. PWM outputs are set to failsafe values.
         on|off      Turn on or off
    
       terminatefail Enable Termination Failsafe mode. While this is true, any
                     failsafe that occurs will be unrecoverable (even if recovery
                     conditions are met).
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
         [-g <val>]  Select channels by group (eg. 0, 1, 2. use 'pwm info' to show
                     groups)
         [-a]        Select all channels
    
     These parameters apply to all commands:
         [-d <val>]  Select PWM output device
                     values: <file:dev>, default: /dev/pwm_output0
         [-v]        Verbose output
         [-e]        Exit with 1 instead of 0 on error
    

## reboot

원본: [systemcmds/reboot](https://github.com/PX4/Firmware/tree/master/src/systemcmds/reboot)

시스템을 다시 부팅합니다

### 사용법 {#reboot_usage}

    reboot [<인자값>...]
         [-b]        부트로더로 다시 부팅
         [lock|unlock] 시스템 끄기 잠금 설정/해제 (시험용)
    

## sd_bench

원본: [systemcmds/sd_bench](https://github.com/PX4/Firmware/tree/master/src/systemcmds/sd_bench)

SD 카드의 속도를 시험합니다.

### 사용법 {#sd_bench_usage}

    sd_bench [arguments...]
         [-b <val>]  Block size for each read/write
                     default: 4096
         [-r <val>]  Number of runs
                     default: 5
         [-d <val>]  Duration of a run in ms
                     default: 2000
         [-s]        Call fsync after each block (default=at end of each run)
    

## top

원본: [systemcmds/top](https://github.com/PX4/Firmware/tree/master/src/systemcmds/top)

실행 프로세스, CPU 활용, 스택 활용, 우선순위, 상태를 출력합니다.

### 사용법 {#top_usage}

    top [<인자값>...]
       once          한 번만 불러온 값 출력
    

## usb_connected

원본: [systemcmds/usb_connected](https://github.com/PX4/Firmware/tree/master/src/systemcmds/usb_connected)

USB 연결 여부를 검사하는 유틸리티입니다. 시작 스크립트에서 활용했습니다. 반환 값 0은 USB 연결함, 1은 이외의 모든 경우에 해당합니다.

### 사용법 {#usb_connected_usage}

    usb_connected [<인자값>...]
    

## ver

원본: [systemcmds/ver](https://github.com/PX4/Firmware/tree/master/src/systemcmds/ver)

다양한 버전 정보를 출력하는 도구입니다

### 사용법 {#ver_usage}

    ver <command> [arguments...]
     Commands:
       hw            Hardware architecture
    
       mcu           MCU info
    
       git           git version information
    
       bdate         Build date and time
    
       gcc           Compiler info
    
       bdate         Build date and time
    
       px4guid       PX4 GUID
    
       uri           Build URI
    
       all           Print all versions
    
       hwcmp         Compare hardware version (returns 0 on match)
         <hw> [<hw2>] Hardware to compare against (eg. PX4_FMU_V4). An OR comparison
                     is used if multiple are specified
    
       hwtypecmp     Compare hardware type (returns 0 on match)
         <hwtype> [<hwtype2>] Hardware type to compare against (eg. V2). An OR
                     comparison is used if multiple are specified