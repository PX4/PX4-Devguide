# 模块参考：估计器

## ekf2

源码：[modules/ekf2](https://github.com/PX4/Firmware/tree/master/src/modules/ekf2)

### 描述

基于扩展卡尔曼滤波器的姿态和位置估计器。 该模块同时应用于多旋翼和固定翼飞机。

更详细的记录可以在 [tuning_the_ecl_ekf](https://dev.px4.io/en/tutorials/tuning_the_ecl_ekf.html) 页面找到。

ekf2 可以在回放（replay）模式中启动 (`-r`)：改模式下它不会访问系统时间，而是使用传感器主题中的时间戳。

### 用法 {#ekf2_usage}

    ekf2 &lt;command&gt; [arguments...]
     Commands:
       start
         [-r]        启用 replay 模式
    
       stop
    
       status        打印状态信息
    

## local_position_estimator

源码：[modules/local_position_estimator](https://github.com/PX4/Firmware/tree/master/src/modules/local_position_estimator)

### 描述

使用扩展卡尔曼滤波器估计姿态和位置信息。

### 用法 {#local_position_estimator_usage}

    local_position_estimator &lt;command&gt; [arguments...]
     Commands:
       start
    
       stop
    
       status        打印状态信息
    

## wind_estimator

源码：[modules/wind_estimator](https://github.com/PX4/Firmware/tree/master/src/modules/wind_estimator)

### 描述

This module runs a combined wind and airspeed scale factor estimator. If provided the vehicle NED speed and attitude it can estimate the horizontal wind components based on a zero sideslip assumption. This makes the estimator only suitable for fixed wing vehicles. If provided an airspeed measurement this module also estimates an airspeed scale factor based on the following model: measured_airspeed = scale_factor * real_airspeed.

### 用法 {#wind_estimator_usage}

    wind_estimator &lt;command&gt; [arguments...]
     Commands:
       start
    
       stop
    
       status        打印状态信息