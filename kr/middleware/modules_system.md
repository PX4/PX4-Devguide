# Modules 레퍼런스: System
## dataman
소스: [modules/dataman](https://github.com/PX4/Firmware/tree/master/src/modules/dataman)


### 설명
C API를 통해서 간단한 데이터베이스 형태로 시스템에 고정 저장소를 제공하는 모듈입니다.
여러 백엔드를 지원:
- 파일 a file (예로 SD 카드)
- FLASH (보드가 지원하는 경우)
- FRAM
- RAM (고정 아님)

다른 타입의 구조화된 데이터를 저정하는데 사용 : mission waypoints, mission state와 geofence polygons.
각 타입은 특정 타입과 고정된 최대 저장 아이템 갯수를 가지며 빠른 random 접근이 가능합니다.

### 구현
단일 아이템을 읽고 쓰기는 항상 atomic합니다. 만약 여러 아이템을 atomic하게 읽기/수정하기가 필요한 경우, 아이템 타입마다 `dm_lock`을 통해 추가로 lock이 있어야 합니다.

**DM_KEY_FENCE_POINTS** 와 **DM_KEY_SAFE_POINTS** 아이템: 최초 데이터 엘리먼트는 `mission_stats_entry_s` 구조로 이런 타입에 대한 다양한 아이템들을 저장합니다. 이 아이템들은 항상 하나의 트랜잭션에서 atomic하게 갱신됩니다.(mavliink mission manager로부터) 이 시간동안 네비게이터는 geofence 아이템 lock을 얻으려고 하며, 실패하면 geofence 위반을 검사하지 않습니다.

### 사용법
```
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
```
## land_detector
소스: [modules/land_detector](https://github.com/PX4/Firmware/tree/master/src/modules/land_detector)


### 설명
이 모듈은 자유낙하(freefall), 비행체의 착륙 상태를 감지하며 `vehicle_land_detected` topic을 publish합니다.
각 비행체 타입(멀티로터, 고정익, vtol, ...)은 자기만의 알고리즘을 제공하며 명령 thrust, arming state과 비행체 모션과 같은 여러 상태를 고려해야합니다.

### 구현
모든 타입은 공통 base 클래스로 자신의 클래스를 구현합니다. base 클래스는 특정 상태(착륙, 착륙가능성, 지상 착지)를 가지고 있습니다. 각각의 가능한 상태는 유도된 클래스에서 구현됩니다. 이력과 각각 내부 상태의 고정된 우선순위가 실제 land_detector 상태를 결정합니다.

#### 멀티콥터 Land Detector
**ground_contact**: thrust setpoint와 z방향으로 속도는 GROUND_CONTACT_TRIGGER_TIME_US 시간동안 정의한 threshold보다 낮아야만 합니다. ground_contact가 감지되면, position 제어기가 thrust setpoint를(body x와 y) 끄게 됩니다.

**maybe_landed**: ground_contact는 좀더 엄격한 thrust setpoint 임계값을 가져야하며 수평방향으로 속도는 필요하지 않습니다. trigger 시간은 MAYBE_LAND_TRIGGER_TIME로 정의합니다. maybe_landed가 감지되면 position 제어기가 thrust setpoint을 0으로 설정합니다.

**landed**: maybe_landed는 LAND_DETECTOR_TRIGGER_TIME_US 시간동안 true로 설정되어 있어야 합니다.

이 모듈은 주기적으로 HP work queue에서 동작합니다.

### 사용법
```
land_detector <command> [arguments...]
 Commands:
   start         Start the background task
     fixedwing|multicopter|vtol|rover Select vehicle type

   stop

   status        print status info
```
## load_mon
소스: [modules/load_mon](https://github.com/PX4/Firmware/tree/master/src/modules/load_mon)


### 설명
LP work queue에서 1 Hz로 주기적으로 실행되는 백그라운드 프로세스로 CPU 로드와 RAM 사용량을 계산하고 `cpuload` topic을 publish합니다.

NuttX에서 각 프로세스의 스택 사용법을 확인하고 만약 300byte 아래로 덜어지면 경고가 출력되는데 log 파일에서도 볼 수 있습니다.

### 사용법
```
load_mon <command> [arguments...]
 Commands:
   start         Start the background task

   stop

   status        print status info
```
## logger
소스: [modules/logger](https://github.com/PX4/Firmware/tree/master/src/modules/logger)


### 설명
uORB topics의 설정가능한 집합과 ULog 파일에 시스템 printf 메시지를(`PX4_WARN`와 `PX4_ERR`) 기록하는 system logger. 시스템과 비행 성능을 평가, 튜닝, replay와 crash 분석하는데 사용할 수 있습니다.

2개 백엔드 지원:
- Files: ULog 파일을 파일 시스템에 작성 (SD 카드)
- MAVLink: MAVLink를 통해 ULog 데이터를 클라이언트에게 전송(클라이언트가 이를 지원해야만 가능)

양쪽 백엔드는 활성화시킬 수 있고 동시에 사용할 수 있습니다.

### 구현
2개 thread를 사용 :
- 메인 thread는 고정 rate로 실행(혹은 -p로 시작시키면 topic을 polling)하고 데이터 갱신을 검사
- writer thread, 파일에 데이터를 쓰는 역할

이 사이에는 설정가능한 크기의 쓰기 버퍼가 있습니다. 손실을 피하기 위해서는 크게 잡아야 합니다.

### 예제
즉시 로깅을 시작시키기 위한 일반적인 사용법:
```
logger start -e -t
```
아니면 이미 실행 중:
```
logger on
```

### 사용법
```
logger <command> [arguments...]
 Commands:
   start
     [-m <val>]  Backend mode
                 values: file|mavlink|all, default: all
     [-e]        Enable logging right after start until disarm (otherwise only
                 when armed)
     [-f]        Log until shutdown (implies -e)
     [-t]        Use date/time for naming log directories and files
     [-r <val>]  Log rate in Hz, 0 means unlimited rate
                 default: 280
     [-b <val>]  Log buffer size in KiB
                 default: 12
     [-q <val>]  uORB queue size for mavlink mode
                 default: 14
     [-p <val>]  Poll on a topic instead of running with fixed rate (Log rate
                 and topic intervals are ignored if this is set)
                 values: <topic_name>

   on            start logging now, override arming (logger must be running)

   off           stop logging now, override arming (logger must be running)

   stop

   status        print status info
```
## replay
소스: [modules/replay](https://github.com/PX4/Firmware/tree/master/src/modules/replay)


### 설명
ULog 파일을 replay하는데 사용하는 모듈.

설정에 사용하는 2개 환경 변수가 있습니다 : `replay`는 ULog 파일 이름으로 설정해야만 합니다. - replay되는 log 파일이기 때문입니다. 두번쨰는 모드로 `replay_mode`로 지정합니다 :
- `replay_mode=ekf2`: 특정 EKF2 replay 모드. ekf2 모듈과 함께 사용할 수 있지만 가능하면 최대한 빠르게 replay할 수 있게 합니다.
- 그외 일반 : 이는 어떤 모듈이라도 replay하는데 사용할 수 있습니다. 하지만 replay는 로그가 기록되었던 것과 같은 속도로 실행됩니다.

이 모듈은 일반적으로 uORB publisher rule과 함께 사용되며 어떤 메시지가 replay되어야하는지 지정합니다. replay 모듈은 log 파일에 있는 모든 메시지를 publish할 것입니다. 해당 log에서 파라미터를 적용할 수도 있습니다.

replay 절차는 [System-wide Replay](https://dev.px4.io/en/debug/system_wide_replay.html)를 참고하세요.

### 사용법
```
replay <command> [arguments...]
 Commands:
   start         Start replay, using log file from ENV variable 'replay'

   trystart      Same as 'start', but silently exit if no log file given

   tryapplyparams Try to apply the parameters from the log file

   stop

   status        print status info
```
## send_event
소스: [modules/events](https://github.com/PX4/Firmware/tree/master/src/modules/events)


### 설명
LP work queue에서 하우스키핑 태스크를 수행하기 위해서 주기적으로 실행되는 백그라운드 프로세스입니다.
현재 온도 칼리브레이션 역할만 수행하고 있습니다.

태스크는 CLI나 uORB topics을 통해서 구동시킬 수 있습니다.(MAVLink에서는 vehicle_command을 통해서)

### 사용법
```
send_event <command> [arguments...]
 Commands:
   start         Start the background task

   temperature_calibration Run temperature calibration process
     [-g]        calibrate the gyro
     [-a]        calibrate the accel
     [-b]        calibrate the baro (if none of these is given, all will be
                 calibrated)

   stop

   status        print status info
```
## sensors
소스: [modules/sensors](https://github.com/PX4/Firmware/tree/master/src/modules/sensors)


### 설명
센서 모듈은 전체 시스템에서 핵심이 되는 부분입니다. 드라이버로부터 로우레벨 출력을 가지며 좀더 유용한 형태로 변경하며 다른 시스템에게 publish합니다.

제공하는 기능은 다음을 포함하고 있습니다:
- 센서 드라이버에서 출력을 읽습니다. (`sensor_gyro` 등)
  만약 동일한 타입이 여러개 있다면 투표와 failover 처리를 합니다.
  가능하다며 보드 회전과 온도 칼리브레이션을 적용합니다. 그리고 마지막으로 데이터를 publish합니다; topic 중에 하나가 `sensor_combined`이며 시스템의 여러 부분에서 사용합니다.
- RC 채널 매핑하기 : raw 입력 채널을(`input_rc`) 읽고나서 칼리브레이션을 적용하고 RC 채널을 설정한 채널과 모드 스위치, low-pass 필터에 매핑하고 `rc_channels`과 `manual_control_setpoint`로 publish합니다.
- ADC 드라이버에서(ioctl 인터페이스를 통해) 출력을 읽고 `battery_status`를 publish합니다.
- 파리미터가 변경되거나 시작되면 센서 드라이버가 업데이트된 칼리브레이션 파라미터를(scale & offset) 얻어왔는지 확인합니다. 센서 드라이버는 파라미터 업데이트를 위해서 ioctl 인터페이스를 사용합니다. 이것이 잘 동작하게 하기 위해서 센서 드라이버는 반드시 `sensor`가 시작될 때 이미 실행되고 있어야 합니다.
- preflight 센서가 일정한지 검사를 하고 `sensor_preflight` topic을 publish합니다.

### 구현
자기 thread로 실행하며 현재 선택한 gyro topic을 poll합니다.

### 사용법
```
sensors <command> [arguments...]
 Commands:
   start
     [-h]        Start in HIL mode

   stop

   status        print status info
```
