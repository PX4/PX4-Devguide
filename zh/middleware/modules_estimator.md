# 模块参考：估计器

## ekf2

源码：[modules/ekf2](https://github.com/PX4/Firmware/tree/master/src/modules/ekf2)

### 描述

基于扩展卡尔曼滤波器的姿态和位置估计器。 该模块同时应用于多旋翼和固定翼飞机。

The documentation can be found on the [ECL/EKF Overview & Tuning](https://docs.px4.io/en/advanced_config/tuning_the_ecl_ekf.html) page.

ekf2 可以在回放（replay）模式中启动 (`-r`)：该模式下它不会访问系统时间，而是使用传感器主题中的时间戳。

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

基于扩展卡尔曼滤波器的姿态和位置估计器。

### 用法 {#local_position_estimator_usage}

    local_position_estimator &lt;command&gt; [arguments...]
     Commands:
       start
    
       stop
    
       status        打印状态信息
    

## wind_estimator

源码：[modules/wind_estimator](https://github.com/PX4/Firmware/tree/master/src/modules/wind_estimator)

### 描述

该模块运行一个综合了风速和空速缩放因子的估计器。 如果给该模块提供 NED 速度和姿态信息，该模块可以根据无侧滑假设对环境风的水平分量进行估计。 这就使得该估计器只适用于固定翼无人机。 如果提供的是空速的测量值，那么该模块还会同时根据如下模型估计一个空速的缩放因子： 测量的空速 = 缩放因子 * 实际空速。

### 用法 {#wind_estimator_usage}

    wind_estimator &lt;command&gt; [arguments...]
     Commands:
       start
    
       stop
    
       status        打印状态信息