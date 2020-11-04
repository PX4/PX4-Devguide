# 모듈 참고: 시스템

## battery_simulator

Source: [modules/simulator/battery_simulator](https://github.com/PX4/PX4-Autopilot/tree/master/src/modules/simulator/battery_simulator)

### 설명

### 사용법 {#battery_simulator_usage}

    battery_simulator <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## battery_status

Source: [modules/battery_status](https://github.com/PX4/PX4-Autopilot/tree/master/src/modules/battery_status)

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

Source: [modules/camera_feedback](https://github.com/PX4/PX4-Autopilot/tree/master/src/modules/camera_feedback)

### 설명

### 사용법 {#camera_feedback_usage}

    camera_feedback <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## commander

Source: [modules/commander](https://github.com/PX4/PX4-Autopilot/tree/master/src/modules/commander)

### 설명

commander 모듈에는 모듈 스위칭 기능과 안전 조치 기능을 수반한 상태 머신이 들어있습니다.

### 사용법 {#commander_usage}

    commander <command> [arguments...]
     Commands:
       start
         [-h]        Enable HIL mode
    
       calibrate     Run sensor calibration
         mag|accel|gyro|level|esc|airspeed Calibration type
         quick       Quick calibration (accel only, not recommended)
    
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

Source: [modules/dataman](https://github.com/PX4/PX4-Autopilot/tree/master/src/modules/dataman)

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

Source: [systemcmds/dmesg](https://github.com/PX4/PX4-Autopilot/tree/master/src/systemcmds/dmesg)

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

Source: [modules/esc_battery](https://github.com/PX4/PX4-Autopilot/tree/master/src/modules/esc_battery)

### 설명

이 구현체에서는 전동 변속기 상태 정보를 활용하며, 배터리 상태를 내보냅니다.

### 사용법 {#esc_battery_usage}

    esc_battery <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## gyro_fft

Source: [examples/gyro_fft](https://github.com/PX4/PX4-Autopilot/tree/master/src/examples/gyro_fft)

### 설명

### Usage {#gyro_fft_usage}

    gyro_fft <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## heater

Source: [drivers/heater](https://github.com/PX4/PX4-Autopilot/tree/master/src/drivers/heater)

### 설명

Background process running periodically on the LP work queue to regulate IMU temperature at a setpoint.

This task can be started at boot from the startup scripts by setting SENS_EN_THERMAL or via CLI.

### Usage {#heater_usage}

    heater <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## land_detector

Source: [modules/land_detector](https://github.com/PX4/PX4-Autopilot/tree/master/src/modules/land_detector)

### Description

Module to detect the freefall and landed state of the vehicle, and publishing the `vehicle_land_detected` topic. Each vehicle type (multirotor, fixedwing, vtol, ...) provides its own algorithm, taking into account various states, such as commanded thrust, arming state and vehicle motion.

### Implementation

Every type is implemented in its own class with a common base class. The base class maintains a state (landed, maybe_landed, ground_contact). Each possible state is implemented in the derived classes. A hysteresis and a fixed priority of each internal state determines the actual land_detector state.

#### 멀티콥터 착륙 감지

**ground_contact**: thrust setpoint and velocity in z-direction must be below a defined threshold for time GROUND_CONTACT_TRIGGER_TIME_US. When ground_contact is detected, the position controller turns off the thrust setpoint in body x and y.

**maybe_landed**: it requires ground_contact together with a tighter thrust setpoint threshold and no velocity in the horizontal direction. The trigger time is defined by MAYBE_LAND_TRIGGER_TIME. When maybe_landed is detected, the position controller sets the thrust setpoint to zero.

**landed**: it requires maybe_landed to be true for time LAND_DETECTOR_TRIGGER_TIME_US.

The module runs periodically on the HP work queue.

### Usage {#land_detector_usage}

    land_detector <command> [arguments...]
     Commands:
       start         Start the background task
         fixedwing|multicopter|vtol|rover|airship Select vehicle type
    
       stop
    
       status        print status info
    

## load_mon

Source: [modules/load_mon](https://github.com/PX4/PX4-Autopilot/tree/master/src/modules/load_mon)

### 설명

Background process running periodically on the low priority work queue to calculate the CPU load and RAM usage and publish the `cpuload` topic.

On NuttX it also checks the stack usage of each process and if it falls below 300 bytes, a warning is output, which will also appear in the log file.

### Usage {#load_mon_usage}

    load_mon <command> [arguments...]
     Commands:
       start         Start the background task
    
       stop
    
       status        print status info
    

## logger

Source: [modules/logger](https://github.com/PX4/PX4-Autopilot/tree/master/src/modules/logger)

### Description

System logger which logs a configurable set of uORB topics and system printf messages (`PX4_WARN` and `PX4_ERR`) to ULog files. These can be used for system and flight performance evaluation, tuning, replay and crash analysis.

It supports 2 backends:

- 파일: ULog 파일을 시스템에 기록합니다(SD 카드)
- MAVLink: 이 프로토콜에 ULog 데이터를 실어 클라이언트에 지속적으로 보냅니다(클라이언트에서 MAVLink 프로토콜을 지원해야함).

Both backends can be enabled and used at the same time.

The file backend supports 2 types of log files: full (the normal log) and a mission log. The mission log is a reduced ulog file and can be used for example for geotagging or vehicle management. It can be enabled and configured via SDLOG_MISSION parameter. The normal log is always a superset of the mission log.

### Implementation

The implementation uses two threads:

- 메인 스레드해서는, 고정 주기(또는 -p 로 시작했을 경우 토픽을 폴링 처리)로 실행하며, 데이터 업데이트를 확인합니다.
- 기록 스레드는 데이터를 파일에 기록합니다

In between there is a write buffer with configurable size (and another fixed-size buffer for the mission log). It should be large to avoid dropouts.

### Examples

Typical usage to start logging immediately:

    logger start -e -t
    

Or if already running:

    logger on
    

### Usage {#logger_usage}

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

Source: [drivers/pwm_input](https://github.com/PX4/PX4-Autopilot/tree/master/src/drivers/pwm_input)

### 설명

Measures the PWM input on AUX5 (or MAIN5) via a timer capture ISR and publishes via the uORB 'pwm_input` message.

### Usage {#pwm_input_usage}

    pwm_input <command> [arguments...]
     Commands:
       start
    
       test          prints PWM capture info.
    
       stop
    
       status        print status info
    

## rc_update

Source: [modules/rc_update](https://github.com/PX4/PX4-Autopilot/tree/master/src/modules/rc_update)

### Description

The rc_update module handles RC channel mapping: read the raw input channels (`input_rc`), then apply the calibration, map the RC channels to the configured channels & mode switches, low-pass filter, and then publish as `rc_channels` and `manual_control_setpoint`.

### Implementation

To reduce control latency, the module is scheduled on input_rc publications.

### Usage {#rc_update_usage}

    rc_update <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## replay

Source: [modules/replay](https://github.com/PX4/PX4-Autopilot/tree/master/src/modules/replay)

### 설명

This module is used to replay ULog files.

There are 2 environment variables used for configuration: `replay`, which must be set to an ULog file name - it's the log file to be replayed. The second is the mode, specified via `replay_mode`:

- `replay_mode=ekf2`: EKF2 재현 모드로 설정합니다. ekf2 모듈하고만 값을 사용할 수 있으나, 가능한 한 빠른 재현이 가능합니다.
- 기타 일반: 다른 모듈 재현에 활용할 수 있습니다만, 기록한 로그 속도대로만 재현할 수 있습니다.

The module is typically used together with uORB publisher rules, to specify which messages should be replayed. The replay module will just publish all messages that are found in the log. It also applies the parameters from the log.

The replay procedure is documented on the [System-wide Replay](https://dev.px4.io/master/en/debug/system_wide_replay.html) page.

### Usage {#replay_usage}

    replay <command> [arguments...]
     Commands:
       start         Start replay, using log file from ENV variable 'replay'
    
       trystart      Same as 'start', but silently exit if no log file given
    
       tryapplyparams Try to apply the parameters from the log file
    
       stop
    
       status        print status info
    

## send_event

Source: [modules/events](https://github.com/PX4/PX4-Autopilot/tree/master/src/modules/events)

### 설명

Background process running periodically on the LP work queue to perform housekeeping tasks. It is currently only responsible for tone alarm on RC Loss.

The tasks can be started via CLI or uORB topics (vehicle_command from MAVLink, etc.).

### Usage {#send_event_usage}

    send_event <command> [arguments...]
     Commands:
       start         Start the background task
    
       stop
    
       status        print status info
    

## sensors

Source: [modules/sensors](https://github.com/PX4/PX4-Autopilot/tree/master/src/modules/sensors)

### Description

The sensors module is central to the whole system. It takes low-level output from drivers, turns it into a more usable form, and publishes it for the rest of the system.

The provided functionality includes:

- 센서 드라이버(`sensor_gyro` 등)의 출력을 읽습니다. 다중 동일 형식 출력이 있을 경우, 출력 데이터 중 하나를 뽑아 안전 조치를 수행합니다. 그 후 보드 회전과 온도 보정 값(기능을 켰을 경우)을 적용합니다. 마지막으로 데이터를 내보냅니다. 내보내는 여러 토픽 중 하나는 시스템의 여러 부분에서 활용하는 `sensor_combined` 입니다.
- 매개변수 값이 바뀌었거나 비행체 가동 시작시, 센서 드라이버에서 최신 보정 매개변수 값(계수와 오프셋)을 받았는지 확인합니다. 센서 드라이버는 매개변수 값 업데이트시 ioctl 인터페이스를 활용합니다. 이 동작을 제대로 수행하기 위해 센서 드라이버는 반드시 `센서`를 시작할 때 먼저 동작해야 합니다.
- 센서 무결성을 점검하고 `sensors_status_imu` 토픽을 내보냅니다.

### Implementation

It runs in its own thread and polls on the currently selected gyro topic.

### Usage {#sensors_usage}

    sensors <command> [arguments...]
     Commands:
       start
         [-h]        Start in HIL mode
    
       stop
    
       status        print status info
    

## temperature_compensation

Source: [modules/temperature_compensation](https://github.com/PX4/PX4-Autopilot/tree/master/src/modules/temperature_compensation)

### 설명

The temperature compensation module allows all of the gyro(s), accel(s), and baro(s) in the system to be temperature compensated. The module monitors the data coming from the sensors and updates the associated sensor_thermal_cal topic whenever a change in temperature is detected. The module can also be configured to perform the coeffecient calculation routine at next boot, which allows the thermal calibration coeffecients to be calculated while the vehicle undergoes a temperature cycle.

### Usage {#temperature_compensation_usage}

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

Source: [systemcmds/tune_control](https://github.com/PX4/PX4-Autopilot/tree/master/src/systemcmds/tune_control)

### Description

Command-line tool to control & test the (external) tunes.

Tunes are used to provide audible notification and warnings (e.g. when the system arms, gets position lock, etc.). The tool requires that a driver is running that can handle the tune_control uorb topic.

Information about the tune format and predefined system tunes can be found here: https://github.com/PX4/PX4-Autopilot/blob/master/src/lib/tunes/tune_definition.desc

### Examples

Play system tune #2:

    tune_control play -t 2
    

### Usage {#tune_control_usage}

    tune_control <command> [arguments...]
     Commands:
       play          Play system tune or single note.
         error       Play error tune
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

Source: [systemcmds/work_queue](https://github.com/PX4/PX4-Autopilot/tree/master/src/systemcmds/work_queue)

### Description

Command-line tool to show work queue status.

### Usage {#work_queue_usage}

    work_queue <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info