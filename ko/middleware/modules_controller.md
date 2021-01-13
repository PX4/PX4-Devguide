!REDIRECT "https://docs.px4.io/master/ko/middleware/modules_controller.html"

# 모듈 참고: 조종 장치

## airship_att_control

Source: [modules/airship_att_control](https://github.com/PX4/Firmware/tree/master/src/modules/airship_att_control)

### 설명

이 모듈은 비행체의 자세, 속도 제어부를 구현합니다. 이상적으로 액츄에이터의 제어 메세지 입출력 용도로 자세 설정값 (`vehicle_attitude_setpoint`) 또는 속도 설정값 (`manual_control_setpoint` 토픽의 아크로(acro) 모드) 을 취합니다.

현재 `manual_control_setpoint` 토픽은 액츄에이터에 직접 전달합니다.

### 구현

제어 지연을 최소화하려면, 모듈에서는 IMU 드라이버에서 보낸 gyro 토픽을 직접 폴링 처리해야합니다.

<a id="airship_att_control_usage"></a>

### Usage

    airship_att_control <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## fw_att_control

Source: [modules/fw_att_control](https://github.com/PX4/Firmware/tree/master/src/modules/fw_att_control)

### 설명

fw_att_control is the fixed wing attitude controller.

<a id="fw_att_control_usage"></a>

### Usage

    fw_att_control <command> [arguments...]
     Commands:
       start
         [vtol]      VTOL mode
    
       stop
    
       status        print status info
    

## fw_pos_control_l1

Source: [modules/fw_pos_control_l1](https://github.com/PX4/Firmware/tree/master/src/modules/fw_pos_control_l1)

### 설명

fw_pos_control_l1 is the fixed wing position controller.

<a id="fw_pos_control_l1_usage"></a>

### Usage

    fw_pos_control_l1 <command> [arguments...]
     Commands:
       start
         [vtol]      VTOL mode
    
       stop
    
       status        print status info
    

## mc_att_control

Source: [modules/mc_att_control](https://github.com/PX4/Firmware/tree/master/src/modules/mc_att_control)

### 설명

This implements the multicopter attitude controller. It takes attitude setpoints (`vehicle_attitude_setpoint`) as inputs and outputs a rate setpoint.

The controller has a P loop for angular error

Publication documenting the implemented Quaternion Attitude Control: Nonlinear Quadrocopter Attitude Control (2013) by Dario Brescianini, Markus Hehn and Raffaello D'Andrea Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

<a id="mc_att_control_usage"></a>

### Usage

    mc_att_control <command> [arguments...]
     Commands:
       start
         [vtol]      VTOL mode
    
       stop
    
       status        print status info
    

## mc_pos_control

Source: [modules/mc_pos_control](https://github.com/PX4/Firmware/tree/master/src/modules/mc_pos_control)

### 설명

The controller has two loops: a P loop for position error and a PID loop for velocity error. Output of the velocity controller is thrust vector that is split to thrust direction (i.e. rotation matrix for multicopter orientation) and thrust scalar (i.e. multicopter thrust itself).

The controller doesn't use Euler angles for its work, they are generated only for more human-friendly control and logging.

<a id="mc_pos_control_usage"></a>

### Usage

    mc_pos_control <command> [arguments...]
     Commands:
       start
         [vtol]      VTOL mode
    
       stop
    
       status        print status info
    

## mc_rate_control

Source: [modules/mc_rate_control](https://github.com/PX4/Firmware/tree/master/src/modules/mc_rate_control)

### 설명

This implements the multicopter rate controller. It takes rate setpoints (in acro mode via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

<a id="mc_rate_control_usage"></a>

### Usage

    mc_rate_control <command> [arguments...]
     Commands:
       start
         [vtol]      VTOL mode
    
       stop
    
       status        print status info
    

## navigator

Source: [modules/navigator](https://github.com/PX4/Firmware/tree/master/src/modules/navigator)

### 설명

Module that is responsible for autonomous flight modes. This includes missions (read from dataman), takeoff and RTL. It is also responsible for geofence violation checking.

### 구현

The different internal modes are implemented as separate classes that inherit from a common base class `NavigatorMode`. The member `_navigation_mode` contains the current active mode.

Navigator publishes position setpoint triplets (`position_setpoint_triplet_s`), which are then used by the position controller.

<a id="navigator_usage"></a>

### Usage

    navigator <command> [arguments...]
     Commands:
       start
    
       fencefile     load a geofence file from SD card, stored at etc/geofence.txt
    
       fake_traffic  publishes 4 fake transponder_report_s uORB messages
    
       stop
    
       status        print status info
    

## rover_pos_control

Source: [modules/rover_pos_control](https://github.com/PX4/Firmware/tree/master/src/modules/rover_pos_control)

### 설명

Controls the position of a ground rover using an L1 controller.

Publishes `actuator_controls_0` messages at a constant 250Hz.

### 구현

Currently, this implementation supports only a few modes:

- Full manual: Throttle, yaw가 액추에이터를 통해 직접적으로 제어됩니다.
- 자동 미션: 탐사선이 임무를 수행
- Loiter: 탐사선이 해당 선회반경 이내로 들어간 후 모터를 멈춥니다.

### 예제

CLI usage example:

    rover_pos_control start
    rover_pos_control status
    rover_pos_control stop
    

<a id="rover_pos_control_usage"></a>

### Usage

    rover_pos_control <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## uuv_att_control

Source: [modules/uuv_att_control](https://github.com/PX4/Firmware/tree/master/src/modules/uuv_att_control)

### 설명

Controls the attitude of an unmanned underwater vehicle (UUV).

Publishes `actuator_controls_0` messages at a constant 250Hz.

### 구현

Currently, this implementation supports only a few modes:

- 완전 수동 방식: 좌우 회전, 상하 회전, 방위 회전, 추력 제어 값을 액츄에이터에 직접 전달
- 자동 수행 임무: 무인 수중선(UUV)의 임무 수행

### 예제

CLI usage example:

    uuv_att_control start
    uuv_att_control status
    uuv_att_control stop
    

<a id="uuv_att_control_usage"></a>

### Usage

    uuv_att_control <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## vtol_att_control

Source: [modules/vtol_att_control](https://github.com/PX4/Firmware/tree/master/src/modules/vtol_att_control)

### 설명

fw_att_control is the fixed wing attitude controller.

<a id="vtol_att_control_usage"></a>

### Usage

    vtol_att_control <command> [arguments...]
     Commands:
    
       stop
    
       status        print status info