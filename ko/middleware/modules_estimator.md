# 모듈 참고: 추정자

## AttitudeEstimatorQ

Source: [modules/attitude_estimator_q](https://github.com/PX4/PX4-Autopilot/tree/master/src/modules/attitude_estimator_q)

### 설명

자세 추정자 Q 입니다.

### 사용법 {#AttitudeEstimatorQ_usage}

    AttitudeEstimatorQ <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## airspeed_estimator

Source: [modules/airspeed_selector](https://github.com/PX4/PX4-Autopilot/tree/master/src/modules/airspeed_selector)

### 설명

This module provides a single airspeed_validated topic, containing indicated (IAS), calibrated (CAS), true airspeed (TAS) and the information if the estimation currently is invalid and if based sensor readings or on groundspeed minus windspeed. 다중 "원시" 항속 입력을 지원하는 이 모듈은 오류 감지시 유효 값을 감지하는 센서로 자동 전환합니다. For failure detection as well as for the estimation of a scale factor from IAS to CAS, it runs several wind estimators and also publishes those.

### 사용법 {#airspeed_estimator_usage}

    airspeed_estimator <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## ekf2

Source: [modules/ekf2](https://github.com/PX4/PX4-Autopilot/tree/master/src/modules/ekf2)

### 설명

확장 칼만 필터를 활용한 자세, 위치 추정자입니다. 멀티로터와 고정익 항공기에 사용합니다.

문서는 [ECL/EKF 개요 및 세부 설정](https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html) 페이지에 있습니다.

ekf2는 재현 모드(`-r`)로 시작할 수 있습니다. 이 모드는 시스템 시간에 맞춰 접근할 수는 없지만 센서 토픽에 인가한 타임스탬프를 활용합니다.

### 사용법 {#ekf2_usage}

    ekf2 <command> [arguments...]
     Commands:
       start
         [-r]        Enable replay mode
    
       stop
    
       status        print status info
    

## local_position_estimator

Source: [modules/local_position_estimator](https://github.com/PX4/PX4-Autopilot/tree/master/src/modules/local_position_estimator)

### 설명

확장 칼만 필터를 활용한 자세, 위치 추정자입니다.

### 사용법 {#local_position_estimator_usage}

    local_position_estimator <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## mc_hover_thrust_estimator

Source: [modules/mc_hover_thrust_estimator](https://github.com/PX4/PX4-Autopilot/tree/master/src/modules/mc_hover_thrust_estimator)

### 설명

### 사용법 {#mc_hover_thrust_estimator_usage}

    mc_hover_thrust_estimator <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info