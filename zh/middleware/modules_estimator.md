# 模块参考：估计器

## ekf2

Source: [modules/ekf2](https://github.com/PX4/Firmware/tree/master/src/modules/ekf2)

### 描述

Attitude and position estimator using an Extended Kalman Filter. It is used for Multirotors and Fixed-Wing.

The documentation can be found on the [tuning_the_ecl_ekf](https://dev.px4.io/en/tutorials/tuning_the_ecl_ekf.html) page.

ekf2 can be started in replay mode (`-r`): in this mode it does not access the system time, but only uses the timestamps from the sensor topics.

### 用法 {#ekf2_usage}

    ekf2 &lt;command&gt; [arguments...]
     Commands:
       start
         [-r]        启用 replay 模式
    
       stop
    
       status        打印状态信息
    

## local_position_estimator

Source: [modules/local_position_estimator](https://github.com/PX4/Firmware/tree/master/src/modules/local_position_estimator)

### 描述

Attitude and position estimator using an Extended Kalman Filter.

### 用法 {#local_position_estimator_usage}

    local_position_estimator &lt;command&gt; [arguments...]
     Commands:
       start
    
       stop
    
       status        打印状态信息
    

## wind_estimator

Source: [modules/wind_estimator](https://github.com/PX4/Firmware/tree/master/src/modules/wind_estimator)

### 描述

This module runs a combined wind and airspeed scale factor estimator. If provided the vehicle NED speed and attitude it can estimate the horizontal wind components based on a zero sideslip assumption. This makes the estimator only suitable for fixed wing vehicles. If provided an airspeed measurement this module also estimates an airspeed scale factor based on the following model: measured_airspeed = scale_factor * real_airspeed.

### 用法 {#wind_estimator_usage}

    wind_estimator &lt;command&gt; [arguments...]
     Commands:
       start
    
       stop
    
       status        打印状态信息