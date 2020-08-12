# 모듈 참고: 조종 장치

## airship_att_control

소스 코드: [modules/airship_att_control](https://github.com/PX4/Firmware/tree/master/src/modules/airship_att_control)

### 설명

이 모듈은 비행체의 자세, 속도 제어부를 구현합니다. 이상적으로 액츄에이터의 제어 메세지 입출력 용도로 자세 설정값 (`vehicle_attitude_setpoint`) 또는 속도 설정값 (`manual_control_setpoint` 토픽의 아크로(acro) 모드) 을 취합니다.

현재 `manual_control_setpoint` 토픽은 액츄에이터에 직접 전달합니다.

### 구현

제어 지연을 최소화하려면, 모듈에서는 IMU 드라이버에서 보낸 gyro 토픽을 직접 폴링 처리해야합니다.

### 사용법 {#airship_att_control_usage}

    airship_att_control <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## fw_att_control

소스 코드: [modules/fw_att_control](https://github.com/PX4/Firmware/tree/master/src/modules/fw_att_control)

### 설명

fw_att_control은 고정익 자세 제어 모듈입니다.

### 사용법 {#fw_att_control_usage}

    fw_att_control <command> [arguments...]
     Commands:
       start
         [vtol]      VTOL mode
    
       stop
    
       status        print status info
    

## fw_pos_control_l1

소스 코드: [modules/fw_pos_control_l1](https://github.com/PX4/Firmware/tree/master/src/modules/fw_pos_control_l1)

### 설명

fw_pos_control_l1은 고정익 위치 제어 모듈입니다.

### 사용법 {#fw_pos_control_l1_usage}

    fw_pos_control_l1 <command> [arguments...]
     Commands:
       start
         [vtol]      VTOL mode
    
       stop
    
       status        print status info
    

## mc_att_control

소스 코드: [modules/mc_att_control](https://github.com/PX4/Firmware/tree/master/src/modules/mc_att_control)

### 설명

이 모듈은 멀티콥터 자세 제어를 구현했습니다. 속도 설정값의 입출력 용도로 자세 설정값(`vehicle_attitude_setpoint`)을 취합니다.

각도 오류 보정을 목적으로 P 루프 로직이 들어있습니다.

4차원 행렬 자세 제어관련 참고 논문은 다음과 같습니다: Nonlinear Quadrocopter Attitude Control (2013) by Dario Brescianini, Markus Hehn and Raffaello D'Andrea Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

### 사용법 {#mc_att_control_usage}

    mc_att_control <command> [arguments...]
     Commands:
       start
         [vtol]      VTOL mode
    
       stop
    
       status        print status info
    

## mc_pos_control

소스 코드: [modules/mc_pos_control](https://github.com/PX4/Firmware/tree/master/src/modules/mc_pos_control)

### 설명

이 제어부에는 두 루프 로직이 있습니다. 위치 오류 보정에 P 루프 로직을 활용하고 속도 오류 보정에 PID 루프로직을 활용합니다. 속도 제어부의 출력은 추력 방향(예: 멀티콥터 방향의 회전 행렬)과 추력 스칼라(예: 멀티콥터 추력 자체값)를 분리하는 추력 벡터 값입니다.

제어부는 자체 동작시 오일러 각 값을 사용하지 않으며, 사용자가 좀 더 이해하기 쉬운 제어부, 기록 데이터를 만듭니다.

### 사용법 {#mc_pos_control_usage}

    mc_pos_control <command> [arguments...]
     Commands:
       start
         [vtol]      VTOL mode
    
       stop
    
       status        print status info
    

## mc_rate_control

소스 코드: [modules/mc_rate_control](https://github.com/PX4/Firmware/tree/master/src/modules/mc_rate_control)

### 설명

멀티콥터 속도 제어부를 구현합니다. 액츄에이터 제어 메시지의 입출력 값으로 속도 설정값(`manual_control_setpoint` 토픽의 아크로(acro) 모드)을 취합니다.

제어부에는 각 속도 오류를 보정하는 PID 루프 로직이 들어있습니다.

### 사용법 {#mc_rate_control_usage}

    mc_rate_control <command> [arguments...]
     Commands:
       start
         [vtol]      VTOL mode
    
       stop
    
       status        print status info
    

## navigator

소스 코드: [modules/navigator](https://github.com/PX4/Firmware/tree/master/src/modules/navigator)

### 설명

자동 비행 모드를 담당하는 모듈입니다. 임무(dataman에서 수신), 이륙, RTL 기능이 들어갑니다. 비행 제한 설정 구역 위반 여부 검사도 처리합니다.

### 구현

`NavigatorMode` 공통 기반 클래스에서 상속한 개별 클래스로 다른 내부 모드를 구현했습니다. `_navigation_mode` 구성 변수에는 현재 활성 모드 값이 들어있습니다.

네비게이터에서는 위체 제어부에서 활용할 위치 설정값 3개(`position_setpoint_triplet_s`)를 내보냅니다.

### 사용법 {#navigator_usage}

    navigator <command> [arguments...]
     Commands:
       start
    
       fencefile     load a geofence file from SD card, stored at etc/geofence.txt
    
       fake_traffic  publishes 4 fake transponder_report_s uORB messages
    
       stop
    
       status        print status info
    

## rover_pos_control

소스 코드: [modules/rover_pos_control](https://github.com/PX4/Firmware/tree/master/src/modules/rover_pos_control)

### 설명

L1 조종 장치로 외계 탐사 차량 위치를 제어합니다.

`actuator_controls_0` 메세지를 초당 250번 규칙적으로 내보냅니다.

### 구현

구현체에서는 현재 일부 모드만 지원합니다:

- Full manual: Throttle, yaw가 액추에이터를 통해 직접적으로 제어됩니다.
- Auto mission: 기체가 미션을 수행합니다
- Loiter: 기체가 그 선회반경 이내로 들어간 후 모터를 멈춥니다.

### 예제

명령행 사용 예제는 다음과 같습니다:

    rover_pos_control start
    rover_pos_control status
    rover_pos_control stop
    

### 사용법 {#rover_pos_control_usage}

    rover_pos_control <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## uuv_att_control

소스 코드: [modules/uuv_att_control](https://github.com/PX4/Firmware/tree/master/src/modules/uuv_att_control)

### 설명

무인 수중선(UUV)의 자세를 제어합니다.

`actuator_controls_0` 메세지를 초당 250번 규칙적으로 내보냅니다.

### 구현

구현체에서는 현재 일부 모드만 지원합니다:

- 완전 수동 방식: 좌우 회전, 상하 회전, 방위 회전, 추력 제어 값을 액츄에이터에 직접 전달
- 자동 수행 임무: 무인 수중선(UUV)의 임무 수행

### 예제

명령행 사용 예제는 다음과 같습니다:

    uuv_att_control start
    uuv_att_control status
    uuv_att_control stop
    

### 사용법 {#uuv_att_control_usage}

    uuv_att_control <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## vtol_att_control

소스 코드: [modules/vtol_att_control](https://github.com/PX4/Firmware/tree/master/src/modules/vtol_att_control)

### 설명

fw_att_control은 고정익 자세 제어부입니다.

### 사용법 {#vtol_att_control_usage}

    vtol_att_control <command> [arguments...]
     Commands:
    
       stop
    
       status        print status info