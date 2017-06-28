# Modules Reference: Estimator
## ekf2
소스: [modules/ekf2](https://github.com/PX4/Firmware/tree/master/src/modules/ekf2)


### 설명
Extended Kalman Filter를 사용하는 Attitude와 position estimator. 멀티로터와 고정익에서 사용합니다.

[tuning_the_ecl_ekf](https://dev.px4.io/en/tutorials/tuning_the_ecl_ekf.html)에서 관련 자료를 찾을 수 있습니다.

ekf2는 replay mode(`-r`)에서 구동시킬 수 있음: 이 모드에서 시스템 시간에 접근할 수 없지만 센서 topic으로부터 timestamp만 사용 가능.


### 사용법
```
ekf2 <command> [arguments...]
 Commands:
   start
     [-r]        Enable replay mode

   stop

   status        print status info
```
