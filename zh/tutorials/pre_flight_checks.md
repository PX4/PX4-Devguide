---
translated_page: https://github.com/PX4/Devguide/blob/master/en/tutorials/pre_flight_checks.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# 飞行前传感器和EKF检查

commander模块执行多个传感器飞行前的质量检查和EKF检查，这些检查由COM \_ARM<>参数控制。如果这些检查失败，电机将无法解锁，并产生以下错误信息

* PREFLIGHT FAIL: EKF HGT ERROR（高度错误）
   * 当IMU和高度检测数据不一致时,会产生此错误。
   * 校准加速度计和陀螺仪后重启飞行器, 如果这个错误任然存在,检查高度传感器数据的问题。
   * 此检查由参数COM_ARM_EKF_HGT控制。
* PREFLIGHT FAIL: EKF VEL ERROR（速度错误）
   * 当IMU和GPS速度检测数据不一致时,会产生此错误。 
   * 检查GPS速度数据是否存在不切合实际的数据跳跃,如果GPS品质良好，执行加速机和陀螺仪校准，并重新启动飞行器。
   * 此检查由参数COM_ARM_EKF_VEL控制。
* PREFLIGHT FAIL: EKF HORIZ POS ERROR（水平位置错误）
   * 当IMU和位置测量数据（GPS或外部视觉设备）不一致时，会产生此错误。 
   * 检查位置传感器数据是否存在不切实际的数据跳转。如果数据品质良好，执行加速和陀螺仪校准，并重新启动飞行器。
   * 此检查由参数COM_ARM_EKF_POS控制
* PREFLIGHT FAIL: EKF YAW ERROR（偏航角错误）
   * 当使用陀螺仪数据估计的偏航角和来自磁力计或外部视觉系统的偏航角不一致时，会产生此错误。
   * 检查IMU数据是否存在大的偏航速率偏移量，并检查磁力计调整和校准。
   * 此检查由参数COM\_ARM\_EKF\_POS控制。
* PREFLIGHT FAIL: EKF HIGH IMU ACCEL BIAS（加速度计错误）
   * 当由EKF估计的IMU加速度计偏差过大时，会产生此错误。
   * 此检查由参数COM\_ARM\_EKF\_AB控制。
* PREFLIGHT FAIL: EKF HIGH IMU GYRO BIAS（陀螺仪错误）
   * 当由EKF估计的IMU陀螺仪偏差过大时，会产生此错误。
   * 此检查由参数COM\_ARM\_EKF\_GB控制
* PREFLIGHT FAIL: ACCEL SENSORS INCONSISTENT - CHECK CALIBRATION（加速度不一致）
   * 当来自不同IMU单元的加速度测量值不一致时，会产生此错误信息。
   * 此检查仅适用于具有多个IMU的电路板。
   * 此检查由参数COM\_ARM\_IMU\_ACC控制。
* PREFLIGHT FAIL: GYRO SENSORS INCONSISTENT - CHECK CALIBRATION（角速度不一致）
   * 当来自不同IMU单元的角速率测量不一致时，会产生此错误信息。
   * 此检查仅适用于具有多个IMU的电路板。
   * 此检查由参数COM_ARM_IMU_GYR 控制。

##COM_ARM_WO_GPS
参数COM_ARM_WO_GPS控制是否允许在没有GPS信号的情况下解锁。当不存在GPS信号时，该参数必须设置为0才允许解锁。只有选择的飞行模式不需要GPS，才允许无GPS的解锁。

##COM_ARM_EKF_POS
参数COM_ARM_EKF_POS控制EKF惯性测量和位置参考（GPS或外部视觉）之间允许的最大不一致度。默认值0.5允许差异不超过EKF允许的最大值的50％，并在飞行开始时提供误差增加的一些余量。

##COM_ARM_EKF_VEL
参数COM_ARM_EKF_VEL控制EKF惯性测量和GPS速度测量之间允许的最大不一致度。默认值0.5允许差异不超过EKF允许的最大值的50％，并在飞行开始时提供误差增加的一些余量。

##COM_ARM_EKF_HGT
参数COM_ARM_EKF_HGT控制EKF惯性测量和高度测量（Baro，GPS，测距仪或外部视觉设备）之间允许的最大不一致度。默认值0.5允许差异不超过EKF允许的最大值的50％，并在飞行开始时提供误差增加的一些余量。
##COM_ARM_EKF_YAW
参数COM_ARM_EKF_YAW控制EKF惯性测量和偏航测量（磁力计或外部视觉）之间允许的最大不一致度。默认值0.5允许差异不超过EKF允许的最大值的50％，并在飞行开始时提供误差增加的一些余量。

##COM_ARM_EKF_AB
参数COM_ARM_EKF_AB控制最大允许的EKF估计IMU加速度计偏差。 默认值0.005允许高达0.5 $m/s^2$的加速度偏差。

##COM_ARM_EKF_GB
参数COM_ARM_EKF_GB控制最大允许的EKF估计IMU陀螺仪偏差。默认值为0.00087允许陀螺仪偏置下的开启高达5度/秒。默认值为0.00087允许陀螺仪高达5度/秒的角速度偏差。

##COM_ARM_IMU_ACC
参数COM_ARM_IMU_ACC控制默认用于飞行控制的IMU与其他IMU单元（如果适用）之间的加速度测量之间允许的最大不一致度。

##COM_ARM_IMU_GYR
参数COM_ARM_IMU_GYR控制默认用于飞行控制的IMU和其他IMU单元（如果适用）之间的角速率测量之间允许的最大不一致度。







