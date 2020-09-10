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

모든 객체 형식은 공통 기반 클래스를 상속받아 자체 클래스로 구현했습니다. 기반 클래스에서는 상태를 관리(landed, maybe_landed, ground_contact)합니다. 가능한 각각의 상태는 상속 클래스에서 구현합니다. 이력 현상(hysteresis)과 각 초기 상태의 고정 우선순위는 실제 land_detector 상태로 결정합니다.

#### 멀티콥터 착륙 감지

**ground_contact**: 추력 설정값과 z 방향 속도는 GROUND_CONTACT_TRIGGER_TIME_US 시간동안 설정 임계값 미만이어야 합니다. ground_contact 를 감지하면, 위치 제어 장치가 동체의 x 및 y 축 방향의 추력 설정 값을 끕니다.

**maybe_landed**: 낮은 추력 한계 값과 수평 방향의 0 속도 값을 지닌 ground_contact가 필요합니다. horizontal direction. 계기 시간은 MAYBE_LAND_TRIGGER_TIM으로 정의합니다. maybe_landed를 감지하면 위치 조정 장치가 추력 설정 값을 0으로 잡습니다.

**landed**: 시간 값을 LAND_DETECTOR_TRIGGER_TIME_US로 설정한 maybe_landed가 필요합니다.

모듈은 HP 작업 큐에서 주기적으로 실행합니다.

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

작업 큐에서 낮은 우선순위로 주기적으로 실행하는 백그라운드 프로세스는 CPU 부하와 RAM 사용량을 계산하고 `cpuload` 토픽으로 내보냅니다.

NuttX에서는 각 프로세스의 스택 사용량을 늘 확인하고 300 바이트 이하로 떨어지면, 로그 파일에 나타나는 경고를 출력합니다.

### 사용법 {#load_mon_usage}

    load_mon <command> [arguments...]
     Commands:
       start         Start the background task
    
       stop
    
       status        print status info
    

## logger

소스 코드: [modules/logger](https://github.com/PX4/Firmware/tree/master/src/modules/logger)

### 설명

여러 uORB 토픽과 시스템 printf 메세지를 설정할 수 있는 항목을 기록하는 시스템 로거입니다. 시스템 및 비행 성능 분석, 세부 설정, 재현, 지명 오류 분석에 활용할 수 있습니다.

백엔드 두가지를 지원합니다:

- 파일: ULog 파일을 시스템에 기록합니다(SD 카드)
- MAVLink: 이 프로토콜에 ULog 데이터를 실어 클라이언트에 지속적으로 보냅니다(클라이언트에서 MAVLink 프로토콜을 지원해야함).

두 백엔드는 동시에 활성화하고 사용할 수 있습니다.

파일 백엔드는 로그 파일 형식 두가지 전체 (일반 로그) 방식, 그리고 임부 기록 방식을 지원합니다. 임무 로그는 축약형 ULog 파일이며, 지리 정보 표시(geotagging) 및 기체 관리 등에 활용할 수 있습니다. SDLOG_MISSION 매개변수로 로그 기능을 켜고 설정할 수 있습니다. 일반 로그는 항상 임무 로그의 상위 집합입니다.

### 구현

구현체에서는 스레드 두가지를 활용합니다:

- 메인 스레드해서는, 고정 주기(또는 -p 로 시작했을 경우 토픽을 폴링 처리)로 실행하며, 데이터 업데이트를 확인합니다.
- 기록 스레드는 데이터를 파일에 기록합니다

이 사이에 설정할 수 있는 크기를 지닌 기록 버퍼가 있습니다(그리고 임무 기록용으로 다른 고정 크기 버퍼도 있음). 기록이 버려지는 일을 막기 위해 버퍼는 충분히 커야합니다.

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

타이머 캡처 ISR을 통해 AUX5 (또는 MAIN5)의 PWM 입력을 측정하고 uORB 'pwm_input' 메세지로 내보냅니다.

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

rc_update 모듈은 RC 채널 대응을 처리합니다. 원시 입력 채널(`input_rc`)의 데이터를 읽고, 보정 값을 적용, RC 채널을 설정 채널, 모드 전환, 저역대 필터에 대응한 다음 `rc_channels`와 `manual_control_setpoint`로 내보냅니다.

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

이 모듈은 ULog 파일 재현에 사용합니다.

설정에 사용하는 환경 변수 2가지가 있습니다. `replay`에는 ULog 파일 이름을 지정해야 하는데, 이 변수에는 재현할 로그 파일이 들어갑니다. 두번째 변수는 `replay_mode`로 지정하는 모드입니다:

- `replay_mode=ekf2`: EKF2 재현 모드로 설정합니다. ekf2 모듈하고만 값을 사용할 수 있으나, 가능한 한 빠른 재현이 가능합니다.
- 기타 일반: 다른 모듈 재현에 활용할 수 있습니다만, 기록한 로그 속도대로만 재현할 수 있습니다.

모듈은 보통 어떤 메세지를 재현해야 하는지 지정하는 uORB 전송 규칙을 함께 활용할 수 있습니다. 재현 모드는 로그에서 찾은 모든 메세지를 내보내기만 합니다. 또한 로그의 매개변수 값을 적용하기도합니다.

재현 절차는 [시스템 수준 재현](https://dev.px4.io/master/en/debug/system_wide_replay.html)페이지에 있습니다.

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

LP 작업 큐에서 백그라운드 프로세스를 주기적으로 실행하여 자원 관리 작업을 수행합니다. 현재는 RC 손실시 소리 알람만 담당합니다.

CLI 또는 uORB 토픽 (MAVLink 의 vehicle_command 등) 을 통해 시작할 수 있습니다.

### 사용법 {#send_event_usage}

    send_event <command> [arguments...]
     Commands:
       start         Start the background task
    
       stop
    
       status        print status info
    

## sensors

소스 코드: [modules/sensors](https://github.com/PX4/Firmware/tree/master/src/modules/sensors)

### 설명

센서 모듈은 전체 시스템의 중심입니다. 드라이버의 저수준 출력, 사용 가능한 구성 형태로의 전환을 취하며, 시스템 나머지 동작 부분을 위해 데이터를 내보냅니다.

제공 기능은 다음과 같습니다:

- 센서 드라이버(`sensor_gyro` 등)의 출력을 읽습니다. 다중 동일 형식 출력이 있을 경우, 출력 데이터 중 하나를 뽑아 안전 조치를 수행합니다. 그 후 보드 회전과 온도 보정 값(기능을 켰을 경우)을 적용합니다. 마지막으로 데이터를 내보냅니다. 내보내는 여러 토픽 중 하나는 시스템의 여러 부분에서 활용하는 `sensor_combined` 입니다.
- 매개변수 값이 바뀌었거나 비행체 가동 시작시, 센서 드라이버에서 최신 보정 매개변수 값(계수와 오프셋)을 받았는지 확인합니다. 센서 드라이버는 매개변수 값 업데이트시 ioctl 인터페이스를 활용합니다. 이 동작을 제대로 수행하기 위해 센서 드라이버는 반드시 `센서`를 시작할 때 먼저 동작해야 합니다.
- 센서 무결성을 점검하고 `sensors_status_imu` 토픽을 내보냅니다.

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

온도 보정 모듈은 시스템의 모든 각가속, 가속, 기압 값에 온도 보정을 적용할 수 있게 합니다. 이 모듈은 센서로 오는 데이터를 확인하고 온도 변화를 감지할 때마다 sensor_thermal_cal 관련 토픽을 업데이트합니다. 이 모듈은 기체가 온도 변화 사이클을 겪는 동안 계산할 온도 보정 상관 계수를 처리할 수 있도록, 다음 부팅시 상관계수 처리 루틴 수행을 설정할 수 있습니다.

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

(외부) 소리를 제어하고 시험하는 명령행 도구입니다.

소리는 들을 수 있는 알림, 경고를 나타낼 때 사용합니다(예: 시스템 이륙 준비, 위치 고정 획득, 등.). 이 도구는 tune_control uORB 토픽을 제어할 수 있는 실행 중인 드라이버가 필요합니다.

소리 형식과 사전 지정 시스템 알림음은 다음 주소에서 찾을 수 있습니다: https://github.com/PX4/Firmware/blob/master/src/lib/tunes/tune_definition.desc

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