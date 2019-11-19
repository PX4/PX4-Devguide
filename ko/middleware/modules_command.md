# 모들 레퍼런스: 명령어

## bl_update

Source: [systemcmds/bl_update](https://github.com/PX4/Firmware/tree/master/src/systemcmds/bl_update)

파일에서 부트로터를 플래시하기 위한 유틸리티

### Usage {#bl_update_usage}

    bl_update [arguments...]
       setopt        Set option bits to unlock the FLASH (only needed if in locked
                     state)
    
       <file>        Bootloader bin file
    

## config

Source: [systemcmds/config](https://github.com/PX4/Firmware/tree/master/src/systemcmds/config)

센서 드라이버 설정 (샘플링 & publication 속도, 범위 등)

### Usage {#config_usage}

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
    

## dumpfile

Source: [systemcmds/dumpfile](https://github.com/PX4/Firmware/tree/master/src/systemcmds/dumpfile)

파일을 덤프하는 유틸리티. 바이너리모드에서 파일 크기와 내용을 출력한다 (LF/CRLF를 변경하지는 않음).

### Usage {#dumpfile_usage}

    dumpfile [arguments...]
         <file>      File to dump
    

## dyn

Source: [systemcmds/dyn](https://github.com/PX4/Firmware/tree/master/src/systemcmds/dyn)

### Description

PX4 바이너리에 컴파일 되지 않은 동적 PX4 모듈을 로드하고 실행한다.

### Example

    dyn ./hello.px4mod start
    

### Usage {#dyn_usage}

    dyn [arguments...]
         <file>      File containing the module
         [arguments...] Arguments to the module
    

## esc_calib

Source: [systemcmds/esc_calib](https://github.com/PX4/Firmware/tree/master/src/systemcmds/esc_calib)

ESC calibration을 위한 툴

Calibration 과정 (명령어를 실행하면 안내가 됩니다):

- ESC의 프로브를 빼고 파워를 끄세요
- Stop attitude and rate controllers: mc_rate_control stop, fw_att_control stop
- safety가 off인지 확인하세요
- 이 명령어를 실행하세요

### Usage {#esc_calib_usage}

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
    

## hardfault_log

Source: [systemcmds/hardfault_log](https://github.com/PX4/Firmware/tree/master/src/systemcmds/hardfault_log)

하드웨어 문제를 다루는 유틸리티

하드웨어문제를 다루는 스타트업 스크립트에 사용됩니다.

### Usage {#hardfault_log_usage}

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

Source: [systemcmds/i2cdetect](https://github.com/PX4/Firmware/tree/master/src/systemcmds/i2cdetect)

특정 버스에 I2C 디바이스를 스캔하기 위한 유틸리티

### Usage {#i2cdetect_usage}

    i2cdetect [arguments...]
         [-b <val>]  I2C bus
                     default: 1
    

## led_control

Source: [systemcmds/led_control](https://github.com/PX4/Firmware/tree/master/src/systemcmds/led_control)

### Description

(외부) LED를 제어하고 테스트하기 위한 CLI 툴

사용하기 위해서는 led_control uORB 토픽을 다루는 드라이버가 수행중인지 확인해야합니다.

우선순위가 있습니다. 예를 들어, 한 모듈이 낮은 우선 순위를 컬러를 출력하고 다른 모듈이 높은 우선 순위로 N번 깜빡이도록 한다면 LED는 깜빡인 후에 컬러가 변경될 것입니다. `reset` 명령어 또한 낮은 우선수위로 돌아가기 위해 사용될 수 있습니다.

### Examples

첫번째 LED를 파란색으로 5회 깜빡이게:

    led_control blink -c blue -l 0 -n 5
    

### Usage {#led_control_usage}

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

Source: [systemcmds/topic_listener](https://github.com/PX4/Firmware/tree/master/src/systemcmds/topic_listener)

uORB 토픽을 수신하고 데이터를 콘솔로 출력하기 위한 유틸리티

Ctrl+C, Esc, Q를 누르면 종료됩니다.

### Usage {#listener_usage}

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

Source: [systemcmds/mixer](https://github.com/PX4/Firmware/tree/master/src/systemcmds/mixer)

### Description

ESC 드라이버에 믹서를 로그하거나 추가합니다.

RPi는 상관없지만, NuttX에서는 사용되는 ioctl 드라이버가 지원되어야 합니다.

### Usage {#mixer_usage}

    mixer <command> [arguments...]
     Commands:
       load
         <file:dev> <file> Output device (eg. /dev/pwm_output0) and mixer file
    
       append
         <file:dev> <file> Output device (eg. /dev/pwm_output0) and mixer file
    

## motor_ramp

Source: [systemcmds/motor_ramp](https://github.com/PX4/Firmware/tree/master/src/systemcmds/motor_ramp)

### Description

모터 ramp up 테스트를 위한 어플리케이션

실행하기 전에 기체 컨트롤러가 멈춰져있는지 확인해주세요

    mc_rate_control stop
    fw_att_control stop
    

시작할 때 하나의 백그라운드 작업이 시작되고 몇초(지정된만큼)간 수행된 후 종료됩니다.

### Example

    motor_ramp sine -a 1100 -r 0.5
    

### Usage {#motor_ramp_usage}

    motor_ramp [arguments...]
         ramp|sine|square mode
         [-d <val>]  Pwm output device
                     default: /dev/pwm_output0
         -a <val>    Select minimum pwm duty cycle in usec
         [-b <val>]  Select maximum pwm duty cycle in usec
                     default: 2000
         [-r <val>]  Select motor ramp duration in sec
                     default: 1.0
    
     WARNING: 모터는 최고 속도까지 ramp up 됩니다.
    

## motor_test

Source: [systemcmds/motor_test](https://github.com/PX4/Firmware/tree/master/src/systemcmds/motor_test)

모터를 테스트하기 위한 유틸리티

WARNING: 이 명령어를 수행하려면 모든 프로브를 제거하세요.

### Usage {#motor_test_usage}

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

Source: [systemcmds/mtd](https://github.com/PX4/Firmware/tree/master/src/systemcmds/mtd)

Utility to mount and test partitions (based on FRAM/EEPROM storage as defined by the board)

### Usage {#mtd_usage}

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

Source: [systemcmds/nshterm](https://github.com/PX4/Firmware/tree/master/src/systemcmds/nshterm)

Start an NSH shell on a given port.

This was previously used to start a shell on the USB serial port. Now there runs mavlink, and it is possible to use a shell over mavlink.

### Usage {#nshterm_usage}

    nshterm [arguments...]
         <file:dev>  Device on which to start the shell (eg. /dev/ttyACM0)
    

## param

Source: [systemcmds/param](https://github.com/PX4/Firmware/tree/master/src/systemcmds/param)

### Description

Command to access and manipulate parameters via shell or script.

This is used for example in the startup script to set airframe-specific parameters.

Parameters are automatically saved when changed, eg. with `param set`. They are typically stored to FRAM or to the SD card. `param select` can be used to change the storage location for subsequent saves (this will need to be (re-)configured on every boot).

If the FLASH-based backend is enabled (which is done at compile time, e.g. for the Intel Aero or Omnibus), `param select` has no effect and the default is always the FLASH backend. However `param save/load <file>` can still be used to write to/read from files.

Each parameter has a 'used' flag, which is set when it's read during boot. It is used to only show relevant parameters to a ground control station.

### Examples

Change the airframe and make sure the airframe's default parameters are loaded:

    param set SYS_AUTOSTART 4001
    param set SYS_AUTOCONFIG 1
    reboot
    

### Usage {#param_usage}

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
         [-c]        Show only changed and used params
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

Source: [systemcmds/perf](https://github.com/PX4/Firmware/tree/master/src/systemcmds/perf)

Tool to print performance counters

### Usage {#perf_usage}

    perf [arguments...]
       reset         Reset all counters
    
       latency       Print HRT timer latency histogram
    
     Prints all performance counters if no arguments given
    

## pwm

Source: [systemcmds/pwm](https://github.com/PX4/Firmware/tree/master/src/systemcmds/pwm)

### Description

This command is used to configure PWM outputs for servo and ESC control.

The default device `/dev/pwm_output0` are the Main channels, AUX channels are on `/dev/pwm_output1` (`-d` parameter).

It is used in the startup script to make sure the PWM parameters (`PWM_*`) are applied (or the ones provided by the airframe config if specified). `pwm info` shows the current settings (the trim value is an offset and configured with `PWM_MAIN_TRIMx` and `PWM_AUX_TRIMx`).

The disarmed value should be set such that the motors don't spin (it's also used for the kill switch), at the minimum value they should spin.

Channels are assigned to a group. Due to hardware limitations, the update rate can only be set per group. Use `pwm info` to display the groups. If the `-c` argument is used, all channels of any included group must be included.

The parameters `-p` and `-r` can be set to a parameter instead of specifying an integer: use -p p:PWM_MIN for example.

Note that in OneShot mode, the PWM range [1000, 2000] is automatically mapped to [125, 250].

### Examples

Set the PWM rate for all channels to 400 Hz:

    pwm rate -a -r 400
    

Test the outputs of eg. channels 1 and 3, and set the PWM value to 1200 us:

    pwm arm
    pwm test -c 13 -p 1200
    

### Usage {#pwm_usage}

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

Source: [systemcmds/reboot](https://github.com/PX4/Firmware/tree/master/src/systemcmds/reboot)

Reboot the system

### Reboot the system {#reboot_usage}

    Usage
    

## sd_bench

Source: [systemcmds/sd_bench](https://github.com/PX4/Firmware/tree/master/src/systemcmds/sd_bench)

Test the speed of an SD Card

### Usage {#sd_bench_usage}

    sd_bench [arguments...]
         [-b <val>]  Block size for each read/write
                     default: 4096
         [-r <val>]  Number of runs
                     default: 5
         [-d <val>]  Duration of a run in ms
                     default: 2000
         [-s]        Call fsync after each block (default=at end of each run)
    

## top

Source: [systemcmds/top](https://github.com/PX4/Firmware/tree/master/src/systemcmds/top)

Monitor running processes and their CPU, stack usage, priority and state

### Usage {#top_usage}

    top [arguments...]
       once          print load only once
    

## usb_connected

Source: [systemcmds/usb_connected](https://github.com/PX4/Firmware/tree/master/src/systemcmds/usb_connected)

Utility to check if USB is connected. Was previously used in startup scripts. A return value of 0 means USB is connected, 1 otherwise.

### Usage {#usb_connected_usage}

    usb_connected [arguments...]
    

## ver

Source: [systemcmds/ver](https://github.com/PX4/Firmware/tree/master/src/systemcmds/ver)

Tool to print various version information

### Usage {#ver_usage}

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
    

## voxlpm

Source: [drivers/power_monitor/voxlpm](https://github.com/PX4/Firmware/tree/master/src/drivers/power_monitor/voxlpm)

### Usage {#voxlpm_usage}

    voxlpm [arguments...]
       start         start monitoring
    
       info          display info
    
       -X            PX4_I2C_BUS_EXPANSION
    
       -T            PX4_I2C_BUS_EXPANSION1
    
       -R            PX4_I2C_BUS_EXPANSION2 (default)