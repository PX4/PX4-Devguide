# 模块参考：控制器

## fw_att_control

Source: [modules/fw_att_control](https://github.com/PX4/Firmware/tree/master/src/modules/fw_att_control)

### 描述

fw_att_control is the fixed wing attitude controller.

### 用法 {#fw_att_control_usage}

    fw_att_control <command> [arguments...]
     Commands:
    
       stop
    
       status        打印状态信息
    

## fw_pos_control_l1

Source: [modules/fw_pos_control_l1](https://github.com/PX4/Firmware/tree/master/src/modules/fw_pos_control_l1)

### 描述

fw_pos_control_l1 is the fixed wing position controller.

### 用法 {#fw_pos_control_l1_usage}

    fw_pos_control_l1 <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        打印状态信息
    

## mc_att_control

Source: [modules/mc_att_control](https://github.com/PX4/Firmware/tree/master/src/modules/mc_att_control)

### 描述

This implements the multicopter attitude and rate controller. It takes attitude setpoints (`vehicle_attitude_setpoint`) or rate setpoints (in acro mode via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has two loops: a P loop for angular error and a PID loop for angular rate error.

Publication documenting the implemented Quaternion Attitude Control: Nonlinear Quadrocopter Attitude Control (2013) by Dario Brescianini, Markus Hehn and Raffaello D'Andrea Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

### Implementation

To reduce control latency, the module directly polls on the gyro topic published by the IMU driver.

### 用法 {#mc_att_control_usage}

    mc_att_control <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        打印状态信息
    

## mc_pos_control

Source: [modules/mc_pos_control](https://github.com/PX4/Firmware/tree/master/src/modules/mc_pos_control)

### 描述

The controller has two loops: a P loop for position error and a PID loop for velocity error. Output of the velocity controller is thrust vector that is split to thrust direction (i.e. rotation matrix for multicopter orientation) and thrust scalar (i.e. multicopter thrust itself).

The controller doesn't use Euler angles for its work, they are generated only for more human-friendly control and logging.

### 用法 {#mc_pos_control_usage}

    mc_pos_control <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        打印状态信息
    

## navigator

Source: [modules/navigator](https://github.com/PX4/Firmware/tree/master/src/modules/navigator)

### 描述

Module that is responsible for autonomous flight modes. This includes missions (read from dataman), takeoff and RTL. It is also responsible for geofence violation checking.

### Implementation

The different internal modes are implemented as separate classes that inherit from a common base class `NavigatorMode`. The member `_navigation_mode` contains the current active mode.

Navigator publishes position setpoint triplets (`position_setpoint_triplet_s`), which are then used by the position controller.

### 用法 {#navigator_usage}

    navigator <command> [arguments...]
     Commands:
       start
    
       fencefile     load a geofence file from SD card, stored at etc/geofence.txt
    
       fake_traffic  publishes 3 fake transponder_report_s uORB messages
    
       stop
    
       status        print status info