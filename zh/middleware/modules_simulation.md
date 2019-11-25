# 模块参考：仿真

## sih

Source: [modules/sih](https://github.com/PX4/Firmware/tree/master/src/modules/sih)

### 说明

本模块为四旋翼提供了一个完全在飞控内部运行的模拟器。

该模拟器订阅了主题“actuator_outputs”，即混控器给出的控制执行器的pwm信号。

This simulator publishes the sensors signals corrupted with realistic noise in order to incorporate the state estimator in the loop.

### Implementation

The simulator implements the equations of motion using matrix algebra. Quaternion representation is used for the attitude. Forward Euler is used for integration. Most of the variables are declared global in the .hpp file to avoid stack overflow.

### Usage {#sih_usage}

    sih <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info