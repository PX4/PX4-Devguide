# 모듈 참고: 추정자

## AttitudeEstimatorQ

소스 코드: [modules/attitude_estimator_q](https://github.com/PX4/Firmware/tree/master/src/modules/attitude_estimator_q)

### 설명

자세 추정자 Q 입니다.

### 사용법 {#AttitudeEstimatorQ_usage}

    AttitudeEstimatorQ <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## airspeed_estimator

소스 코드: [modules/airspeed_selector](https://github.com/PX4/Firmware/tree/master/src/modules/airspeed_selector)

### 설명

이 모듈은 계기 항속(IAS), 등가 항속(EAS), 실제 항속(TAS)과 현재 추정치가 잘못되었는지 여부의 정보, 센서 기반 데이터 또는 지상 속도에 풍속을 뺀 속도 데이터에 기반하는지 여부의 정보가 들어간 단일 airspeed_validated 토픽을 내보냅니다. 다중 "원시" 항속 입력을 지원하는 이 모듈은 오류 감지시 유효 값을 감지하는 센서로 자동 전환합니다. For failure detection as well as for the estimation of a scale factor from IAS to EAS, it runs several wind estimators and also publishes those.

### 사용법 {#airspeed_estimator_usage}

    airspeed_estimator <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## ekf2

Source: [modules/ekf2](https://github.com/PX4/Firmware/tree/master/src/modules/ekf2)

### Description

Attitude and position estimator using an Extended Kalman Filter. It is used for Multirotors and Fixed-Wing.

The documentation can be found on the [ECL/EKF Overview & Tuning](https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html) page.

ekf2 can be started in replay mode (`-r`): in this mode it does not access the system time, but only uses the timestamps from the sensor topics.

### Usage {#ekf2_usage}

    ekf2 <command> [arguments...]
     Commands:
       start
         [-r]        Enable replay mode
    
       stop
    
       status        print status info
    

## local_position_estimator

Source: [modules/local_position_estimator](https://github.com/PX4/Firmware/tree/master/src/modules/local_position_estimator)

### Description

Attitude and position estimator using an Extended Kalman Filter.

### Usage {#local_position_estimator_usage}

    local_position_estimator <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## mc_hover_thrust_estimator

Source: [modules/mc_hover_thrust_estimator](https://github.com/PX4/Firmware/tree/master/src/modules/mc_hover_thrust_estimator)

### Description

### Usage {#mc_hover_thrust_estimator_usage}

    mc_hover_thrust_estimator <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info