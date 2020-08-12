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

이 제어부에는 두 루프 로직이 있습니다. 위치 오류 보정에 P 루프를 활용하고 속도 오류 보정에 PID 루프를 활용합니다. Output of the velocity controller is thrust vector that is split to thrust direction (i.e. rotation matrix for multicopter orientation) and thrust scalar (i.e. multicopter thrust itself).

The controller doesn't use Euler angles for its work, they are generated only for more human-friendly control and logging.

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

This implements the multicopter rate controller. It takes rate setpoints (in acro mode via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

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

Module that is responsible for autonomous flight modes. This includes missions (read from dataman), takeoff and RTL. It is also responsible for geofence violation checking.

### 구현

The different internal modes are implemented as separate classes that inherit from a common base class `NavigatorMode`. The member `_navigation_mode` contains the current active mode.

Navigator publishes position setpoint triplets (`position_setpoint_triplet_s`), which are then used by the position controller.

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

Controls the position of a ground rover using an L1 controller.

Publishes `actuator_controls_0` messages at a constant 250Hz.

### 구현

Currently, this implementation supports only a few modes:

- Full manual: Throttle, yaw가 액추에이터를 통해 직접적으로 제어됩니다.
- Auto mission: 기체가 미션을 수행합니다
- Loiter: 기체가 그 선회반경 이내로 들어간 후 모터를 멈춥니다.

### 예제

CLI usage example:

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

Controls the attitude of an unmanned underwater vehicle (UUV).

Publishes `actuator_controls_0` messages at a constant 250Hz.

### 구현

Currently, this implementation supports only a few modes:

- Full manual: Roll, pitch, yaw, and throttle controls are passed directly through to the actuators
- Auto mission: The uuv runs missions

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

fw_att_control is the fixed wing attitude controller.

### 사용법 {#vtol_att_control_usage}

    vtol_att_control <command> [arguments...]
     Commands:
    
       stop
    
       status        print status info