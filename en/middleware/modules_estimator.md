# Modules Reference: Estimator
## ekf2
Source: [modules/ekf2](https://github.com/PX4/Firmware/tree/master/src/modules/ekf2)


### Description
Attitude and position estimator using an Extended Kalman Filter. It is used for Multirotors and Fixed-Wing.

The documentation can be found on the [tuning_the_ecl_ekf](https://dev.px4.io/en/tutorials/tuning_the_ecl_ekf.html) page.

ekf2 can be started in replay mode (`-r`): in this mode it does not access the system time, but only uses the
timestamps from the sensor topics.


### Usage {#ekf2_usage}
```
ekf2 <command> [arguments...]
 Commands:
   start
     [-r]        Enable replay mode

   stop

   status        print status info
```
