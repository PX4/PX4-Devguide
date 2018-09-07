# Modules Reference: Estimator

## ekf2

Source: [modules/ekf2](https://github.com/PX4/Firmware/tree/master/src/modules/ekf2)

### Description

Attitude and position estimator using an Extended Kalman Filter. It is used for Multirotors and Fixed-Wing.

The documentation can be found on the [tuning_the_ecl_ekf](https://dev.px4.io/en/tutorials/tuning_the_ecl_ekf.html) page.

ekf2 can be started in replay mode (`-r`): in this mode it does not access the system time, but only uses the timestamps from the sensor topics.

### Usage {#ekf2_usage}

    ekf2 <command> [arguments...]
     Commands:
       start
         [-r]        Enable replay mode
    
       stop
    
       status        print status info
    

## local_position_estimator

Source: [modules/local_position_estimator](https://github.com/PX4/Firmware/tree/master/src/modules/local_position_estimator)

### Description

Attitude and position estimator using an Extended Kalman Filter.

### Usage {#local_position_estimator_usage}

    local_position_estimator <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## wind_estimator

Source: [modules/wind_estimator](https://github.com/PX4/Firmware/tree/master/src/modules/wind_estimator)

### Description

This module runs a combined wind and airspeed scale factor estimator. If provided the vehicle NED speed and attitude it can estimate the horizontal wind components based on a zero sideslip assumption. This makes the estimator only suitable for fixed wing vehicles. If provided an airspeed measurement this module also estimates an airspeed scale factor based on the following model: measured_airspeed = scale_factor * real_airspeed.

### Usage {#wind_estimator_usage}

    wind_estimator <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info