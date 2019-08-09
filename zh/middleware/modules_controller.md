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

为了降低控制延时，该模块会直接轮训 IMU 驱动发布的陀螺仪（gyro）主题消息。

### 用法 {#mc_att_control_usage}

    mc_att_control <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        打印状态信息
    

## mc_pos_control

源码：[modules/mc_pos_control](https://github.com/PX4/Firmware/tree/master/src/modules/mc_pos_control)

### 描述

控制器有两个回路：一个针对位置误差的比例（P）控制回路和一个针对速度误差的 PID 控制回路。 速度控制器的输出是一个推力矢量，该矢量可分割成推力的方向（即，多旋翼姿态的旋转矩阵）和推力的大小（即，多旋翼推力本身）

控制器实际工作中并不使用欧拉角，系统运行时使用欧拉角仅仅是为了可以更人性化地控制飞机和日志记录。

### 用法 {#mc_pos_control_usage}

    mc_pos_control <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        打印状态信息
    

## navigator

源码：[modules/navigator](https://github.com/PX4/Firmware/tree/master/src/modules/navigator)

### 描述

该模块负责自主飞行模式。 这里面包括了飞行任务 (从 dataman 中读取)，起飞和 RTL。 它还负责检查飞机是否跨越了地理围栏。

### 实现

不同的内部模式都是以单独的类实现的，这些类都是从公共基类 `NavigatorMode` 的子类。 成员变量 `_navigation_mode` 包含了当前活跃的模式。

Navigator 发布位置期望值三元组 (`position_setpoint_triplet_s`)，该期望值会被位置控制器使用。

### 用法 {#navigator_usage}

    navigator &lt;command&gt; [arguments...]
     Commands:
       start
    
       fencefile     从 SD 卡载入地理围栏文件，文件位置：etc/geofence.txt
    
       fake_traffic  发布 3 个虚假的 transponder_report_s uORB 消息
    
       stop
    
       status        打印状态信息
    

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