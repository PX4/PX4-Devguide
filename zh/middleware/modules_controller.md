# 模块参考：控制器

## fw_att_control

源码：[modules/fw_att_control](https://github.com/PX4/Firmware/tree/master/src/modules/fw_att_control)

### 描述

fw_att_control 是固定翼姿态控制器。

### 用法 {#fw_att_control_usage}

    fw_att_control <command> [arguments...]
     Commands:
    
       stop
    
       status        打印状态信息
    

## fw_pos_control_l1

源码：[modules/fw_pos_control_l1](https://github.com/PX4/Firmware/tree/master/src/modules/fw_pos_control_l1)

### 描述

fw_pos_control_l1 是针对固定翼飞机的位置控制器。

### 用法 {#fw_pos_control_l1_usage}

    fw_pos_control_l1 <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        打印状态信息
    

## mc_att_control

源码：[modules/mc_att_control](https://github.com/PX4/Firmware/tree/master/src/modules/mc_att_control)

### 描述

此模块实现了多旋翼无人机的姿态角和姿态角速率控制器。 该模块以姿态角期望值 (`vehicle_attitude_setpoint`) 或者姿态角速率期望值 (在竞技（acro）模式下通过`manual_control_setpoint` 主题发布该指令) 作为输入然后输出执行器控制量消息。

控制器有两个回路：一个针对角度误差的比例（P）控制回路和一个针对角速率误差的 PID 控制回路。

记载了实现基于四元数的姿态控制的出版物：Nonlinear Quadrocopter Attitude Control (2013) by Dario Brescianini, Markus Hehn and Raffaello D'Andrea Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

### 实现

To reduce control latency, the module directly polls on the gyro topic published by the IMU driver.

### 用法 {#mc_att_control_usage}

    mc_att_control <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        打印状态信息
    

## mc_pos_control

源码：[modules/mc_pos_control](https://github.com/PX4/Firmware/tree/master/src/modules/mc_pos_control)

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

源码：[modules/navigator](https://github.com/PX4/Firmware/tree/master/src/modules/navigator)

### 描述

Module that is responsible for autonomous flight modes. This includes missions (read from dataman), takeoff and RTL. It is also responsible for geofence violation checking.

### 实现

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