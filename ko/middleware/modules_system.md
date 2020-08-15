# 모듈 참고: 시스템

## battery_simulator

소스 코드: [modules/simulator/battery_simulator](https://github.com/PX4/Firmware/tree/master/src/modules/simulator/battery_simulator)

### 설명

### 사용법 {#battery_simulator_usage}

    battery_simulator <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## battery_status

소스 코드: [modules/battery_status](https://github.com/PX4/Firmware/tree/master/src/modules/battery_status)

### 설명

제공 기능은 다음과 같습니다:

- (ioctl 인터페이스로) ADC 드라이버의 출력을 읽고 `battery_status`로 내보냅니다.

### 구현

현재 선택한 자이로 토픽에 대해 스레드와 폴링을 기반으로 실행합니다.

### 사용법 {#battery_status_usage}

    battery_status <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## camera_feedback

소스 코드: [modules/camera_feedback](https://github.com/PX4/Firmware/tree/master/src/modules/camera_feedback)

### 설명

### 사용법 {#camera_feedback_usage}

    camera_feedback <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## commander

소스 코드: [modules/commander](https://github.com/PX4/Firmware/tree/master/src/modules/commander)

### 설명

commander 모듈에는 모듈 스위칭 기능과 안전 조치 기능을 수반한 상태 머신이 들어있습니다.

### 사용법 {#commander_usage}

    commander <command> [arguments...]
     Commands:
       start
         [-h]        Enable HIL mode
    
       calibrate     Run sensor calibration
         mag|accel|gyro|level|esc|airspeed Calibration type
    
       check         Run preflight checks
    
       arm
         [-f]        Force arming (do not run preflight checks)
    
       disarm
    
       takeoff
    
       land
    
       transition    VTOL transition
    
       mode          Change flight mode
         manual|acro|offboard|stabilized|rattitude|altctl|posctl|auto:mission|auto:l
                     oiter|auto:rtl|auto:takeoff|auto:land|auto:precland Flight mode
    
       lockdown
         [off]       Turn lockdown off
    
       stop
    
       status        print status info
    

## dataman

소스 코드: [modules/dataman](https://github.com/PX4/Firmware/tree/master/src/modules/dataman)

### 설명

C API 형식의 간단한 데이터베이스형 영구 저장소를 시스템의 나머지 자원을 가용하여 제공합니다. 여러 백엔드를 지원합니다:

- 파일(예: SD 카드) 
- 플래시(보드에 붙어있을 경우)
- FRAM
- RAM(분명히 영구 저장장치는 아님)

임부 경로 지점, 임무 상태, 다각형 비행 제한 구역 같은 여러가지 형식의 구조 데이터를 저장합니다. 각 데이터에 고유 형식을 할당하며, 항목당 저장소상 최대 고정 용량이 있기에, 고속의 임의 접근이 가능합니다.

### 구현

단일 항목의 읽기/쓰기 동작은 원소 형태입니다. 여러 항목을 개별적으로 읽고 수정하려면, `dm_lock`으로 항목 형식별 추가 잠금을 걸어두어야 합니다.

**DM_KEY_FENCE_POINTS**와 **DM_KEY_SAFE_POINTS** 항목: 첫 데이타 요소는 이런 형식의 항목을 저장하는 `mission_stats_entry_s` 구조체입니다. 트랜잭션 1회당 이 항목은 (MAVLink 임무 관리자에서) 개별적으로 업데이트합니다. 이때 동안, 네비게이터에서는 비행 제한 구역(geofence) 항목의 잠금 획득을 시도하고, 비행 제한 구역 위반을 검사하지 않습니다.

### 사용법 {#dataman_usage}

    dataman <command> [arguments...]
     Commands:
       start
         [-f <val>]  Storage file
                     values: <file>
         [-r]        Use RAM backend (NOT persistent)
         [-i]        Use FLASH backend
    
     The options -f, -r and -i are mutually exclusive. If nothing is specified, a
     file 'dataman' is used
    
       poweronrestart Restart dataman (on power on)
    
       inflightrestart Restart dataman (in flight)
    
       stop
    
       status        print status info
    

## dmesg

소스 코드: [systemcmds/dmesg](https://github.com/PX4/Firmware/tree/master/src/systemcmds/dmesg)

### 설명

부팅 콘솔 메시지를 보여주는 명령행 도구입니다. NuttX의 작업 큐와 syslog의 출력은 잡아 보여주지 않습니다.

### 예제

백그라운드의 모든 메시지 출력 유지:

    dmesg -f &
    

### 사용법 {#dmesg_usage}

    dmesg <command> [arguments...]
     Commands:
         [-f]        Follow: wait for new messages
    

## esc_battery

소스 코드: [modules/esc_battery](https://github.com/PX4/Firmware/tree/master/src/modules/esc_battery)

### 설명

이 구현체에서는 ESC 상태 정보를 활용하며, 배터리 상태를 내보냅니다.

### 사용법 {#esc_battery_usage}

    esc_battery <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## heater

소스 코드: [drivers/heater](https://github.com/PX4/Firmware/tree/master/src/drivers/heater)

### 설명

백그라운드 프로세스는 관성센서의 온도를 지정 값으로 정규화하려는 목적으로 LP 작업 큐에서 주기적으로 실행합니다.

이 작업은 SENS_EN_THERMAL을 설정하여 부팅할 때 시작 스크립트로 시작하거나 명령행 환경에서 시작할 수 있습니다.

### 사용법 {#heater_usage}

    heater <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## land_detector

소스 코드: [modules/land_detector](https://github.com/PX4/Firmware/tree/master/src/modules/land_detector)

### 설명

기체의 자유 낙하를 감지하거나 착륙 상태를 감지하는 모듈이며, `vehicle_land_detected` 토픽을 내보냅니다. 여러가지 기체 형식(멀티로터, 고정익, 수직이착륙기,...) 에 대해 고유의 알고리즘을 제공하며, 명령에 의한 추진, 이륙 준비 상태, 기체의 움직임 등 여러 상태를 고려합니다.

### 구현

Every type is implemented in its own class with a common base class. The base class maintains a state (landed, maybe_landed, ground_contact). Each possible state is implemented in the derived classes. A hysteresis and a fixed priority of each internal state determines the actual land_detector state.

#### 멀티콥터 착륙 감지

**ground_contact**: thrust setpoint and velocity in z-direction must be below a defined threshold for time GROUND_CONTACT_TRIGGER_TIME_US. When ground_contact is detected, the position controller turns off the thrust setpoint in body x and y.

**maybe_landed**: it requires ground_contact together with a tighter thrust setpoint threshold and no velocity in the horizontal direction. The trigger time is defined by MAYBE_LAND_TRIGGER_TIME. When maybe_landed is detected, the position controller sets the thrust setpoint to zero.

**landed**: it requires maybe_landed to be true for time LAND_DETECTOR_TRIGGER_TIME_US.

The module runs periodically on the HP work queue.

### 사용법 {#land_detector_usage}

    land_detector <command> [arguments...]
     Commands:
       start         Start the background task
         fixedwing|multicopter|vtol|rover|airship Select vehicle type
    
       stop
    
       status        print status info
    

## load_mon

소스 코드: [modules/load_mon](https://github.com/PX4/Firmware/tree/master/src/modules/load_mon)

### 설명

Background process running periodically on the low priority work queue to calculate the CPU load and RAM usage and publish the `cpuload` topic.

On NuttX it also checks the stack usage of each process and if it falls below 300 bytes, a warning is output, which will also appear in the log file.

### 사용법 {#load_mon_usage}

    load_mon <command> [arguments...]
     Commands:
       start         Start the background task
    
       stop
    
       status        print status info
    

## logger

소스 코드: [modules/logger](https://github.com/PX4/Firmware/tree/master/src/modules/logger)

### Description

System logger which logs a configurable set of uORB topics and system printf messages (`PX4_WARN` and `PX4_ERR`) to ULog files. These can be used for system and flight performance evaluation, tuning, replay and crash analysis.

It supports 2 backends:

- Files: write ULog files to the file system (SD card)
- MAVLink: stream ULog data via MAVLink to a client (the client must support this)

Both backends can be enabled and used at the same time.

The file backend supports 2 types of log files: full (the normal log) and a mission log. The mission log is a reduced ulog file and can be used for example for geotagging or vehicle management. It can be enabled and configured via SDLOG_MISSION parameter. The normal log is always a superset of the mission log.

### 구현

The implementation uses two threads:

- The main thread, running at a fixed rate (or polling on a topic if started with -p) and checking for data updates
- The writer thread, writing data to the file

In between there is a write buffer with configurable size (and another fixed-size buffer for the mission log). It should be large to avoid dropouts.

### 예제

보통 로깅을 바로 시작할 때 사용법:

    logger start -e -t
    

또는 이미 동작중일 경우:

    logger on
    

### 사용법 {#logger_usage}

    logger <command> [arguments...]
     Commands:
       start
         [-m <val>]  Backend mode
                     values: file|mavlink|all, default: all
         [-x]        Enable/disable logging via Aux1 RC channel
         [-e]        Enable logging right after start until disarm (otherwise only
                     when armed)
         [-f]        Log until shutdown (implies -e)
         [-t]        Use date/time for naming log directories and files
         [-r <val>]  Log rate in Hz, 0 means unlimited rate
                     default: 280
         [-b <val>]  Log buffer size in KiB
                     default: 12
         [-p <val>]  Poll on a topic instead of running with fixed rate (Log rate
                     and topic intervals are ignored if this is set)
                     values: <topic_name>
    
       on            start logging now, override arming (logger must be running)
    
       off           stop logging now, override arming (logger must be running)
    
       stop
    
       status        print status info
    

## pwm_input

소스 코드: [drivers/pwm_input](https://github.com/PX4/Firmware/tree/master/src/drivers/pwm_input)

### 설명

Measures the PWM input on AUX5 (or MAIN5) via a timer capture ISR and publishes via the uORB 'pwm_input` message.

### 사용법 {#pwm_input_usage}

    pwm_input <command> [arguments...]
     Commands:
       start
    
       test          prints PWM capture info.
    
       stop
    
       status        print status info
    

## rc_update

소스 코드: [modules/rc_update](https://github.com/PX4/Firmware/tree/master/src/modules/rc_update)

### 설명

The rc_update module handles RC channel mapping: read the raw input channels (`input_rc`), then apply the calibration, map the RC channels to the configured channels & mode switches, low-pass filter, and then publish as `rc_channels` and `manual_control_setpoint`.

### 구현

제어 지연을 줄이려, 모듈은 input_rc 를 내보낼 때 동작하도록 했습니다.

### 사용법 {#rc_update_usage}

    rc_update <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## replay

소스 코드: [modules/replay](https://github.com/PX4/Firmware/tree/master/src/modules/replay)

### 설명

This module is used to replay ULog files.

There are 2 environment variables used for configuration: `replay`, which must be set to an ULog file name - it's the log file to be replayed. The second is the mode, specified via `replay_mode`:

- `replay_mode=ekf2`: specific EKF2 replay mode. It can only be used with the ekf2 module, but allows the replay to run as fast as possible.
- Generic otherwise: this can be used to replay any module(s), but the replay will be done with the same speed as the log was recorded.

The module is typically used together with uORB publisher rules, to specify which messages should be replayed. The replay module will just publish all messages that are found in the log. It also applies the parameters from the log.

The replay procedure is documented on the [System-wide Replay](https://dev.px4.io/master/en/debug/system_wide_replay.html) page.

### 사용법 {#replay_usage}

    replay <command> [arguments...]
     Commands:
       start         Start replay, using log file from ENV variable 'replay'
    
       trystart      Same as 'start', but silently exit if no log file given
    
       tryapplyparams Try to apply the parameters from the log file
    
       stop
    
       status        print status info
    

## send_event

소스 코드: [modules/events](https://github.com/PX4/Firmware/tree/master/src/modules/events)

### 설명

Background process running periodically on the LP work queue to perform housekeeping tasks. It is currently only responsible for tone alarm on RC Loss.

The tasks can be started via CLI or uORB topics (vehicle_command from MAVLink, etc.).

### 사용법 {#send_event_usage}

    send_event <command> [arguments...]
     Commands:
       start         Start the background task
    
       stop
    
       status        print status info
    

## sensors

소스 코드: [modules/sensors](https://github.com/PX4/Firmware/tree/master/src/modules/sensors)

### 설명

The sensors module is central to the whole system. It takes low-level output from drivers, turns it into a more usable form, and publishes it for the rest of the system.

The provided functionality includes:

- Read the output from the sensor drivers (`sensor_gyro`, etc.). If there are multiple of the same type, do voting and failover handling. Then apply the board rotation and temperature calibration (if enabled). And finally publish the data; one of the topics is `sensor_combined`, used by many parts of the system.
- Make sure the sensor drivers get the updated calibration parameters (scale & offset) when the parameters change or on startup. The sensor drivers use the ioctl interface for parameter updates. For this to work properly, the sensor drivers must already be running when `sensors` is started.
- Do preflight sensor consistency checks and publish the `sensor_preflight` topic.

### 구현

현재 선택한 자이로 토픽에 대해 스레드와 폴링을 기반으로 실행합니다.

### 사용법 {#sensors_usage}

    sensors <command> [arguments...]
     Commands:
       start
         [-h]        Start in HIL mode
    
       stop
    
       status        print status info
    

## temperature_compensation

소스 코드: [modules/temperature_compensation](https://github.com/PX4/Firmware/tree/master/src/modules/temperature_compensation)

### 설명

The temperature compensation module allows all of the gyro(s), accel(s), and baro(s) in the system to be temperature compensated. The module monitors the data coming from the sensors and updates the associated sensor_thermal_cal topic whenever a change in temperature is detected. The module can also be configured to perform the coeffecient calculation routine at next boot, which allows the thermal calibration coeffecients to be calculated while the vehicle undergoes a temperature cycle.

### 사용법 {#temperature_compensation_usage}

    temperature_compensation <command> [arguments...]
     Commands:
       start         Start the module, which monitors the sensors and updates the
                     sensor_thermal_cal topic
    
       calibrate     Run temperature calibration process
         [-g]        calibrate the gyro
         [-a]        calibrate the accel
         [-b]        calibrate the baro (if none of these is given, all will be
                     calibrated)
    
       stop
    
       status        print status info
    

## tune_control

소스 코드: [systemcmds/tune_control](https://github.com/PX4/Firmware/tree/master/src/systemcmds/tune_control)

### 설명

Command-line tool to control & test the (external) tunes.

Tunes are used to provide audible notification and warnings (e.g. when the system arms, gets position lock, etc.). The tool requires that a driver is running that can handle the tune_control uorb topic.

Information about the tune format and predefined system tunes can be found here: https://github.com/PX4/Firmware/blob/master/src/lib/tunes/tune_definition.desc

### 예제

시스템 소리 2번을 재생하려면:

    tune_control play -t 2
    

### 사용법 {#tune_control_usage}

    tune_control <command> [arguments...]
     Commands:
       play          Play system tune or single note.
         [-t <val>]  Play predefined system tune
                     default: 1
         [-f <val>]  Frequency of note in Hz (0-22kHz)
         [-d <val>]  Duration of note in us
         [-s <val>]  Volume level (loudness) of the note (0-100)
                     default: 40
         [-m <val>]  Melody in string form
                     values: <string> - e.g. "MFT200e8a8a"
    
       libtest       Test library
    
       stop          Stop playback (use for repeated tunes)
    

## work_queue

소스 코드: [systemcmds/work_queue](https://github.com/PX4/Firmware/tree/master/src/systemcmds/work_queue)

### 설명

작업 큐 상태를 나타내는 명령행 도구입니다.

### 사용법 {#work_queue_usage}

    work_queue <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info