# Modules Reference: Controller

## fw_att_control

Source: [modules/fw_att_control](https://github.com/PX4/Firmware/tree/master/src/modules/fw_att_control)

### Description

fw_att_control는 고정익 기체 컨트롤러입니다.

### Usage {#fw_att_control_usage}

    fw_att_control <command> [arguments...]
     Commands:
    
       stop
    
       status        print status info
    

## fw_pos_control_l1

Source: [modules/fw_pos_control_l1](https://github.com/PX4/Firmware/tree/master/src/modules/fw_pos_control_l1)

### Description

fw_pos_control_l1는 고정익 날개 위치 컨트롤러입니다.

### Usage {#fw_pos_control_l1_usage}

    fw_pos_control_l1 <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## mc_att_control

Source: [modules/mc_att_control](https://github.com/PX4/Firmware/tree/master/src/modules/mc_att_control)

### Description

멀티콥터의 자세와 속도 컨트롤러를 구현한 것입니다. 입력으로 자세 설정값(`vehicle_attitude_setpoint`) 또는 속도 설정값 (acro 모드에서 `manual_control_setpoint` 토픽을 통해)을 받아들이고 액추테이터의 컨트롤 메시지를 출력합니다.

컨트롤러는 2개의 루프를 갖고 있습니다. 각도 에러에 대한 루프, 각도 속도 에러에 대한 루프입니다.

Quaternion Attitude Control 구현에 대한 문서 : Nonlinear Quadrocopter Attitude Control (2013) by Dario Brescianini, Markus Hehn and Raffaello D'Andrea Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

### Implementation

이 모듈을 컨트롤 지연을 줄이기 위해 IMU 드라이버에 의해 퍼블리쉬된 gyro topic에 직접적으로 풀링합니다.

### Usage {#mc_att_control_usage}

    mc_att_control <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## mc_pos_control

Source: [modules/mc_pos_control](https://github.com/PX4/Firmware/tree/master/src/modules/mc_pos_control)

### Description

이 컨트롤러는 2개의 루프를 가지고 있습니다. 위치 에러에 대한 루프, 속도 에러에 대한 루프입니다. 속도 컨트롤러의 출력은 추력의 방향과 (예. 멀티콥터 방향을 위한 회전 행렬) 추력의 양 (예. 멀티콥터 추력)으로 나눠지는 추려 벡터입니다.

The controller doesn't use Euler angles for its work, they are generated only for more human-friendly control and logging.

### Usage {#mc_pos_control_usage}

    mc_pos_control <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## navigator

Source: [modules/navigator](https://github.com/PX4/Firmware/tree/master/src/modules/navigator)

### Description

Module that is responsible for autonomous flight modes. This includes missions (read from dataman), takeoff and RTL. It is also responsible for geofence violation checking.

### Implementation

The different internal modes are implemented as separate classes that inherit from a common base class `NavigatorMode`. The member `_navigation_mode` contains the current active mode.

Navigator publishes position setpoint triplets (`position_setpoint_triplet_s`), which are then used by the position controller.

### Usage {#navigator_usage}

    navigator <command> [arguments...]
     Commands:
       start
    
       fencefile     load a geofence file from SD card, stored at etc/geofence.txt
    
       fake_traffic  publishes 3 fake transponder_report_s uORB messages
    
       stop
    
       status        print status info
    

## rover_pos_control

Source: [modules/rover_pos_control](https://github.com/PX4/Firmware/tree/master/src/modules/rover_pos_control)

### Description

Controls the position of a ground rover using an L1 controller.

Publishes `actuator_controls_0` messages at a constant 250Hz.

### Implementation

Currently, this implementation supports only a few modes:

- Full manual: Throttle and yaw controls are passed directly through to the actuators
- Auto mission: The rover runs missions
- Loiter: The rover will navigate to within the loiter radius, then stop the motors

### Examples

CLI usage example:

    rover_pos_control start
    rover_pos_control status
    rover_pos_control stop
    

### Usage {#rover_pos_control_usage}

    rover_pos_control <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## vtol_att_control

Source: [modules/vtol_att_control](https://github.com/PX4/Firmware/tree/master/src/modules/vtol_att_control)

### Description

fw_att_control is the fixed wing attitude controller.

### Usage {#vtol_att_control_usage}

    vtol_att_control <command> [arguments...]
     Commands:
    
       stop
    
       status        print status info