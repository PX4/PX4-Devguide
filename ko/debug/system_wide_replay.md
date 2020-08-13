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
- ORB 전송 규칙을 `build/px4_sitl_default_replay/tmp/rootfs/orb_publisher.rules`에 추가하십시오. This file defines which module is allowed to publish which messages. It has the following format:
    
        restrict_topics: <topic1>, <topic2>, ..., <topicN>
        module: <module>
        ignore_others: <true/false>
        
    
    It means that the given list of topics should only be published by `<module>` (which is the command name). Publications to any of these topics from another module are silently ignored. If `ignore_others` is `true`, then publications to other topics from `<module>` are ignored.
    
    For replay, we only want the `replay` module to be able to publish the previously identified list of topics. So for replaying `ekf2`, the rules file looks like this:
    
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

### Important Notes

- During replay, all dropouts in the log file are reported. These have a negative effect on replay and thus it should be taken care that dropouts are avoided during recording.
- It is currently only possible to replay in 'real-time', meaning as fast as the recording was done. This is planned to be extended in the future.
- A message that has a timestamp of 0 will be considered invalid and not be replayed.

## EKF2 재현

This is a specialization of the system-wide replay for fast EKF2 replay. It will automatically create the ORB publisher rules and works as following:

- Optionally set `SDLOG_MODE` to 1 to start logging from boot
- Record the log
- To replay:

    export replay_mode=ekf2
    export replay=<abs_path_to_log.ulg>
    make px4_sitl none
    

You can stop it after there's an output like:

    INFO  [replay] Replay done (published 9917 msgs, 2.136 s)
    

The parameters can be adjusted as well. They can be extracted from the log with \(install pyulog with `sudo pip install pyulog` first\):

    ulog_params -i "$replay" -d ' ' | grep -e '^EKF2' > build/px4_sitl_default_replay/tmp/rootfs/replay_params.txt
    

Then edit the parameters in the file as needed and restart the replay process with `make px4_sitl none`. This will create a new log file.

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