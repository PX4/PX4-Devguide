# EKF2 Log Replay

여기서는 실제 flight log에 replay 기능을 사용해서 EKF2 estimator의 파라미터를 튜닝하는 방법을 보여줍니다.

## 소개
개발자는 estimation 분석을 위해서 특정 로그 데이터를 replay하는 것이 가능합니다. 이 페이지의 나머지 부분은 효과를 보기 위해서 어떤 파라미터를 설정해야하는지 그리고 올바르게 사용하는 방법에 대해서 알아봅니다.

## sdlog2 logger (.px4log)

### 전제 조건
*  **SYS_LOGGER** 파라미터를 sdlog2(디폴트)로 설정하고 기체를 리부팅합니다.(0 = sdlog2 이고 1 = ulog)
* **EKF2_REC_RPL** 파라미터를 1로 설정. 이렇게 하면 estimator로 하여금 logging에서 특정 replay 메시지를 publish하도록 합니다.
* **SDLOG_PRIO_BOOST** parameter를 {0, 1, 2, 3} 중에 하나의 값으로 설정합니다. 0인 경우는 온보드 logging app은 기본적으로 \(low\) 스케쥴링 우선순위를 가지게 됩니다. low 스케쥴링 우선순위인 경우 logging 메시지 손실이 일어날 수 있습니다. log 파일에 건너뛴 메시지가 있는 경우 'gaps'이라고 표시되므로 이런 경우 parameter 값을 최대치인 3으로 설정합니다. 테스팅에서는 데이터 손실을 피하기 위해서 최소 2 값으로 설정합니다.

### 배포

일단 위에 설정으로 생성된 실제 flight log(.px4log)가 있다면, PX4 펌웨어의 root 디렉토리에서 다음과 같은 명령을 이용해서 replay를 **실행** 시킬 수 있습니다. :
```
make posix_sitl_replay replay logfile=<absolute_path_to_log_file>/my_log_file.px4log
```

해당 명령이 실행되면 replay할 로그 파일의 위치와 이름을 검사합니다.
이 파일은 다음 위치에 있어야 합니다.
```
<path to Firmware>/build/posix_sitl_replay/src/firmware/posix/rootfs/
```
출력 replay 파일을 **replay_replayed.px4log** 라고 부릅니다. estimator 성능을 분석하는데 사용할 수 있습니다.


### replay를 위해 튜닝 파라미터 변경
처음으로 replay를 실행할 때, replay_replayed.px4log 파일은 실제 비행에서 디폴트 EKF2 파라미터 값을 사용해서 생성됩니다. 이후에 **replay\_params.txt** 에 있는 어떤 EKF2 파라미터도 변경할 수 있으며 output 파일처럼 동일한 디렉토리에 위치하고 있습니다.   

예로 gyro bias를 위해 노이즈 값 설정은 다음 라인을 필요로 합니다.

```
EKF2_GB_NOISE 0.001
```
일단 일부 EKF2 파라미터가 변경되면, [배포](#deployment)에서 주어진 동일한 명령을 사용해서 새로운 replay_replayed.px4log 파일을 생성할 수 있습니다.
