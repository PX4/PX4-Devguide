# 시스템 범위 재현

ORB 메세지에 기반하여, 시스템에 붙어있는 각 부품의 동작을 기록하고 재현할 수 있습니다.

실제 데이터를 기반으로 한 제각각의 매개변수 값 영향 시험, 제각기 다른 추정자의 동작 비교 등에 재현 과정에 상당히 쓸만합니다.

## 준비 요건

가장 먼저 필요한 과정은 재현할 모듈의 파악입니다. 그 다음 해당 모듈의 입력단을 모두 확인합니다(예: 지속 수신하는 ORB 토픽). 시스템 범위 재현시, 모든 하드웨어 입력요소로, 센서, 원격 조종 입력, MAVLink 명령, 파일 시스템을 들 수 있습니다.

파악한 모든 토픽을 최대 데이터 전송률로 기록해야합니다([로깅](../log/logging.md) 참고). `ekf2`의 경우 이미 토픽 로깅 기본 설정에 반영했습니다.

모든 재현 토픽에는 `timestamp` 필드에 자동으로 입력하는 단일 타임스탬프 절대값이 들어있다는 점이 중요합니다. 타임스탬프 정보가 더 들어가야 하겠지만, 그렇다면 메인 타임스탬프 값에 상대적인 값이 들어가야 합니다. 예제는 [sensor_combined.msg](https://github.com/PX4/Firmware/blob/master/msg/sensor_combined.msg)에 있습니다. 이 경우에 대한 이유는 아래에 설명해드리겠습니다.

## 사용법

- 우선 재현할 파일을 선택하고, 대상을 빌드하십시오(Firmware 디렉터리에서 처리함): 
        sh
        export replay=<absolute_path_to_log_file.ulg>
        make px4_sitl_default 별도의 빌드 디렉터리 
    
    `build/px4_sitl_default_replay` 에 출력 내용을 새로 만들어둡니다(그래서 기존 빌드와 매개변수의 값이 뒤섞이지 않음). 재현을 위해 임의의 POSIX SITL 빌드 대상을 선택할 수 있습니다. 빌드 시스템에서는 `replay` 환경 변수를 통해 재현 모드에 들어갔음을 이해합니다.
- ORB 전송 규칙을 `build/px4_sitl_default_replay/tmp/rootfs/orb_publisher.rules`에 추가하십시오. 이 파일에는 어떤 모듈에서 어떤 메세지를 내보낼지 여부 설정값이 들어있습니다. 파일의 설정 형식은 다음과 같습니다:
    
        restrict_topics: <topic1>, <topic2>, ..., <topicN>
        module: <module>
        ignore_others: <true/false>
        
    
    위 기록은 `<module>`(명령 이름)에서 내보낼 여러 토픽을 나타냅니다. 다른 모듈에서 내보내는 토픽은 조용히 무시합니다. `ignore_others` 값을 `true`로 설정하면 다른 `<module>`에서 내보낸 토픽은 무시합니다.
    
    재현을 위해 `replay` 모듈에서 앞서 식별한 토픽 목록을 내보내려합니다. So for replaying `ekf2`, the rules file looks like this:
    
        restrict_topics: sensor_combined, vehicle_gps_position, vehicle_land_detected
        module: replay
        ignore_others: true
        
    
    This allows that the modules, which usually publish these topics, don't need to be disabled for replay.

- Optional: setup parameter overrides in the file `build/px4_sitl_default_replay/tmp/rootfs/replay_params.txt`. This file should contain a list of `<param_name> <value>`, like:
    
        EKF2_GB_NOISE 0.001
        
    
    By default, all parameters from the log file are applied. When a parameter changed during recording, it will be changed as well at the right time during replay. A parameter in the `replay_params.txt` will override the value and changes to it from the log file will not be applied.

- Optional: copy `dataman` missions file from the SD card to the build directory. Only necessary if a mission should be replayed.
- Start the replay:
    
    ```sh
    make px4_sitl_default jmavsim
    ```
    
    This will automatically open the log file, apply the parameters and start to replay. Once done, it will be reported and the process can be exited. Then the newly generated log file can be analyzed, it has `_replayed` appended to its file name.
    
    Note that the above command will show the simulator as well, but depending on what is being replayed, it will not show what's actually going on. It's possible to connect via QGC and e.g. view the changing attitude during replay.

- Finally, unset the environment variable, so that the normal build targets are used again:
    
    ```sh
    unset replay
    ```

### 중요사항

- 재현 과정에서 버리는 기록 내용 보고 내용이 나타납니다. 재현 과정에 있어 부정적인 현상이며, 기록을 진행하는 동안 일부 기록을 폐기하는 일이 없도록 주의깊게 살펴보아야 합니다.
- 현재는 '실시간' 재현만 가능합니다. 한마디로, 기록을 재빠르게 진행한 만큼 재현도 동일한 시간에 진행한다는 의미입니다. 관련 기능은 나중에 좀 더 추가하겠습니다.
- 타임스탬프가 0으로 찍힌 메세지는 잘못된 메세지로 간주하며 재현 과정에 반영하지 않습니다.

## EKF2 재현

이 절의 내용은 EKF2의 시스템 범위 고속 재현에 해당합니다. ORB 전송 규칙을 자동으로 만들며 다음과 같이 동작합니다:

- `SDLOG_MODE`를 별도로 1로 설정하여 부팅할 때 로깅을 시작하도록 함
- 여러 동작과 상태를 기록
- 이 과정을 재현하려면:

    export replay_mode=ekf2
    export replay=<abs_path_to_log.ulg>
    make px4_sitl none
    

다음과 같은 출력 내용이 뜨고 나면 중단할 수 있습니다:

    INFO  [replay] Replay done (published 9917 msgs, 2.136 s)
    

매개변수도 마찬가지로 조정할 수 있습니다. 다음 명령으로 로그에서 매개변수 값을 뽑아낼 수 있습니다 \(우선 `sudo pip install pyulog` 명령으로 pyulog를 설치하십시오\):

    ulog_params -i "$replay" -d ' ' | grep -e '^EKF2' > build/px4_sitl_default_replay/tmp/rootfs/replay_params.txt
    

그 다음 원하는대로 파일의 매개변수 값을 편집한 후 `make px4_sitl none` 명령으로 재현 과정을 다시 시작하십시오. 이 과정을 통해 새 로그 파일을 만듭니다.

생성 로그 위치는 다음 메세지처럼 화면에 나타납니다:

    INFO  [logger] Opened log file: rootfs/fs/microsd/log/2017-03-01/13_30_51_replayed.ulg
    

과정이 끝난 후 재현 모드를 끝내려면 `unset replay; unset replay_mode` 명령을 내리십시오.

## Behind the Scenes

Replay is split into 3 components:

- a replay module
- ORB publisher rules
- time handling

The replay module reads the log and publishes the messages with the same speed as they were recorded. A constant offset is added to the timestamp of each message to match the current system time (this is the reason why all other timestamps need to be relative). The command `replay tryapplyparams` is executed before all other modules are loaded and applies the parameters from the log and user-set parameters. Then as the last command, `replay trystart` will again apply the parameters and start the actual replay. Both commands do nothing if the environment variable `replay` is not set.

The ORB publisher rules allow to select which part of the system is replayed, as described above. They are only compiled for the posix SITL targets.

The **time handling** is still an **open point**, and needs to be implemented.