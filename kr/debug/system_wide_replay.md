# 시스템 수준에서 Replay
ORB 메시지를 바탕으로 시스템 특정 부분의 동작을 기록하고 이를 재연하는 것이 가능합니다. 이를 가능하게 하기 위해서 새로운 logger를 활성화시켜야 합니다.(`SYS_LOGGER`를 1로 설정)

재연은 실제 데이터를 기반으로 다른 파라미터 값의 영향을 테스트하는데 효과적입니다. 다른 estimator들을 비교할 수도 있습니다.

## 전제 조건
가장 먼저 해야하는 일은 replay해야 하는 모듈을 구별해내는 것입니다.
다음으로 이 모듈로 들어오는 모든 입력(예로 subscribed ORB topic)을 아는 것입니다. 시스템 수준에서 replay를 위해서는 모든 하드웨어 입력은 즉 센서, RC 입력, mavlink 명령, 파일 시스템으로 구성됩니다.

구별가능한 모든 topic들은 최대 rate로 로그를 남겨야 합니다.([logging](../log/logging.md) 참고) `ekf2` 경우에는 이미 기본적으로 관련 topic에 대해서 로그를 남기고 있습니다.

재연하는 모든 topic은 단일 timestamp를 사용하는 것이 중요합니다. `timestamp` 필드에 자동으로 생성됩니다. 더 많은 timestamp가 있어야 한다면, main timestamp와 관련되어 있어야 합니다. 예제로 [sensor_combined.msg](https://github.com/PX4/Firmware/blob/master/msg/sensor_combined.msg)를 참고하세요.
이에 대한 이유는 아래에서 설명합니다.

## 사용

- 먼저 재연할 파일을 선택하고 타겟(Firmware 디렉토리 내에 있는)을 빌드합니다:
```sh
export replay=<absolute_path_to_log_file.ulg>
make px4_sitl_default
```
  여기서는 `build/px4_sitl_default_replay`라는 별도의 빌드 디렉토리에 결과물을 생성합니다. (이 파라미터는 일반 빌드에 영향을 주지 않음) 재연을 위해 posix SITL build target을 선택하는 것이 가능합니다. 빌드 시스템은 `replay` 환경변수를 통해 현재 재연 모드에 있는지 알 수 있습니다.
- ORB publisher rule 파일을
  `build/px4_sitl_default_replay/tmp/rootfs/orb_publisher.rules`에 추가합니다.
  이 파일은 어떤 모듈이 어떤 메시지를 publish하도록 허용하는지 정의합니다.
  다음과 같은 포맷으로 :
```
restrict_topics: <topic1>, <topic2>, ..., <topicN>
module: <module>
ignore_others: <true/false>
```
  주어진 topics 리스트는 반드시 `<module>`(command 이름)로 publish되어야 합니다. 다른 module로부터 이 topic들 중에 publish는 무시됩니다. 만약 `ignore_others`가 `true`라면, `<module>`로부터 다른 topic으로 publish는 무시됩니다.

  replay에 대해서는 `replay` 모듈이 이전에 구분한 topic 목록을 publish할 수 있도록 원합니다. `ekf2`을 replay하는데 있어서 rule 파일은 다음과 같습니다. :
```
restrict_topics: sensor_combined, vehicle_gps_position, vehicle_land_detected
module: replay
ignore_others: true
```

  이렇게 하는 경우, 일반적으로 여기 topic들을 publish하는 module이 replay에 대해서 disable하지 않아도 됩니다.

- Optional: setup parameter를 아래 파일에 덮어쓴다.
  `build/px4_sitl_default_replay/tmp/rootfs/replay_params.txt`.
  이 파일은 `<param_name> <value>`의 목록을 포함해야만 한다. 다음과 같다. :
```
EKF2_GB_NOISE 0.001
```

  기본적으로 log 파일에 모든 parameter가 적용됩니다. parameter가 저장하는 동안 변경되면, replay 하는 동안 해당 타이밍에 변경될 것입니다. `replay_params.txt` 내부에 parameter는 값을 덮어쓰고 log 파일에서 parameter가 변경되는 경우에는 적용되지 않습니다.
- Optional: SD 카드에 있는 `dataman` mission 파일을 해당 build 디렉토리에 복사. mission이 replay인 경우에만 필요.
- replay 시작하기:
```sh
  make px4_sitl_default jmavsim
```
  이렇게 하면 자동으로 log 파일을 열어서 paramter를 적용하고 replay를 시작합니다. 일단 완료되면, 결과를 리포팅하고 해당 process는 종료됩니다. 다음으로 새로 생성된 log 파일은 분석하고 `_replayed`가 파일이름에 추가됩니다.

  위 command는 simulator로도 보여집니다. 하지만 무엇을 replay하느냐에 따라서 실제로 진행되는 것을 보여주지 않을 수도 있습니다. 예를 든다면 QGC 연결을 통해서 가능하며 예를 들자면 replay 동안 attitude 변경을 볼 수 있습니다.

- 마지막으로 환경 변수를 unset하는데 다시 일반 build target을 사용합니다. :
```sh
unset replay
```

### Important Notes

- replay를 하는 동안 log 파일에 있는 모든 dropout을 리포팅합니다. 이렇게하면 replay에 부정적인 영향을 미치므로 recording시에는 dropout이 되지 않도록 신경써야만 합니다.
- 현재 '실시간'으로 replay만 가능해서 recording만큼 빠르게 수행해야 합니다. 이 기능은 향후 확장될 예정입니다.
- timestamp가 0인 message는 유효하지 않은 것으로 간주하며 replay할때 사용되지 않습니다.

## EKF2 Replay

이것은 빠른 EKF2 replay를 위해 시스템 수준의 replay를 특화시킨 것입니다. 자동으로 ORB publisher rule을 생성하고 다음과 같이 동작합니다 :

* 선택적으로 `SDLOG_MODE`를 1로 설정해서 부트에서 로깅을 시작시킴
* log 기록
* replay를 위해:

```
export replay_mode=ekf2
export replay=<abs_path_to_log.ulg>
make posix none
```

다음과 같은 output이 있고나서 정지할 수 있습니다 :

```
INFO  [replay] Replay done (published 9917 msgs, 2.136 s)
```

파라미터도 조절할 수 있습니다. \(먼저 pyulog 설치 `sudo pip install pyulog`\)하고 log로부터 추출할 수 있습니다 :

```
ulog_params -i $replay -d ' ' | grep -e '^EKF2' > build/px4_sitl_default_replay/tmp/rootfs/replay_params.txt
```
다음으로는 필요하면 파일에 있는 파라미터를 수정하고 `make posix none`로 replay 프로세스를 재시작합니다. 이렇게하면 새로운 로그 파일이 생성됩니다.

생성한 로그의 위치는 다음과 같이 메시지와 함께 출력 :

```
INFO  [logger] Opened log file: rootfs/fs/microsd/log/2017-03-01/13_30_51_replayed.ulg
```

종료할 때, replay 모드를 빠져나가기 위해 `unset replay; unset replay_mode`를 사용합니다.

## Behind the Scenes

Replay는 3개 컴포넌트로 분리 :
- replay 모듈
- ORB publisher rules
- time 처리

replay 모듈은 log를 읽고 record될 때와 동일한 속도로 message를 publish 합니다. 일정한 offset을 각 message의 timestamp에 추가하여 현재 system time과 매치(모든 timestamp가 상대적인 값인 이유)시킵니다. `replay tryapplyparams` 명령은 다른 모든 module들이 load되기 전에 실행해야하며 log에 있는 parameter와 user-set parameter를 적용합니다. 다음으로 마지막 명령 `replay trystart`은 다시 parameter를 적용하고 실제 replay를 시작시킵니다. 이 두가지 명령 모두 환경변수 `replay`가 설정되어 있지 않으면 아무런 일도 수행하지 않습니다.

ORB publisher rule은 위에 설명한 것처럼 system의 어느 부분이 replay될 것인지 선택합니다. posix SITL target에서만 컴파일됩니다.

**time 처리** 는 향후 구현이 필요합니다.
