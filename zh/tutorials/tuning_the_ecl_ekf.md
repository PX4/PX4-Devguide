# 使用 ecl EKF

本教程回答了有关使用 ECL EKF 算法的常见问题。

## 什么是 ecl EKF？

估算和控制库（ECL）使用扩展卡尔曼滤波器（EKF）算法来处理传感器测量并提供以下状态的估计：

* 四元数定义从北，东，下局部地球坐标系到 X，Y，Z 体坐标系的旋转
* IMU 的速度 - 北，东，下\（m/s\）
* 在 IMU 的位置 - 北，东，向下\（m\）
* IMU delta 角度偏差估计- X，Y，Z\（rad\）
* IMU delta 速度偏差估计- X，Y，Z\（m/s \）
* 地球磁场组件-北，东，下\（高斯\）
* 车身框架磁场偏置-X，Y，Z\（高斯\）
* 风速-北，东\（m/s\）

EKF 在延迟的“融合时间范围”上运行，以允许相对于 IMU 的每次测量的不同时间延迟。 每个传感器的数据都是 FIFO 缓冲的，并由 EKF 从缓冲区中检索，以便在正确的时间使用。 每个传感器的延迟补偿由 [EKF2 _*_ DELAY](../advanced/parameter_reference.md#ekf2) 参数控制。

互补滤波器用于使用缓冲的 IMU 数据将状态从“融合时间范围”向前传播到当前时间。 该滤波器的时间常数由 [EKF2_TAU_VEL](../advanced/parameter_reference.md#EKF2_TAU_VEL) 和 [EKF2_TAU_POS](../advanced/parameter_reference.md#EKF2_TAU_POS) 参数控制。

> **Note** '融合时间范围'延迟和缓冲区长度由最大的 EKF2\_\*\_DELAY 参数决定。 如果未使用传感器，建议将其时间延迟设置为零。 减少“融合时间范围”延迟减少了用于将状态向前传播到当前时间的互补滤波器中的误差。

调整位置和速度状态以考虑 IMU 和机身框架在输出到控制回路之前的偏移。 IMU 相对于主体框架的位置由 `EKF2\_IMU\_POS\_X，Y，Z` 参数设置。

EKF 仅使用 IMU 数据进行状态预测。 IMU 数据不用作 EKF 推导中的观察。 使用 Matlab 符号工具箱导出协方差预测，状态更新和协方差更新的代数方程，可在此处找到：[Matlab 符号推导](https://github.com/PX4/ecl/blob/master/EKF/matlab/scripts/Inertial Nav EKF/GenerateNavFilterEquations.m)。

## 它使用什么传感器测量？

EKF 具有不同的操作模式，允许不同的传感器测量组合。 在启动时，过滤器检查传感器的最小可行组合，并且在初始倾斜，偏航和高度对准完成之后，进入提供旋转，垂直速度，垂直位置，IMU 德角偏差和 IMU 德尔塔速度偏差估计的模式。

此模式需要 IMU 数据，偏航源（磁力计或外部视觉\）和高度数据源。 所有 EKF 操作模式都需要此最小数据集。 然后可以使用其他传感器数据来估计其他状态。

### IMU

* 三轴体固定惯性测量单位 delta 角和 delta 速度数据，最小速率为 100Hz。 在 EKF 使用之前，应将锥形校正应用于 IMU delta 角度数据。

### 磁力计

需要以最小 5Hz 的速率的三轴体固定磁力计数据\（或外部视觉系统姿势数据\）。 磁力计数据可以通过两种方式使用：

* 使用倾斜估计和磁偏角将磁强计测量值转换为偏航角。 然后将该偏航角用作 EKF 的观察。 该方法精度较低并且不允许学习体架场偏移，但是它对于磁异常和大型启动陀螺偏置更有鲁棒性。 它是启动期间和地面使用的默认方法。
* XYZ 磁力计读数用作单独的观察。 该方法更精确并且允许学习体架场偏移，但是假设地球磁场环境仅缓慢变化并且当存在显着的外部磁异常时表现较差。 这是车辆在空中飞行并爬升超过 1.5 米高度时的默认方法。

用于选择模式的逻辑由 [EKF2_MAG_TYPE](../advanced/parameter_reference.md#EKF2_MAG_TYPE) 参数设置。

### 高度

需要高度数据源-GPS，气压，测距仪或外部视觉，最小速率为 5Hz。 [Note](../advanced/parameter_reference.md#EKF2_HGT_MODE) 高度数据的主要来源由 <0>EKF2_HGT_MODE</0> 参数控制。

如果不存在这些测量值，EKF 将无法启动。 当检测到这些测量值时，EKF 将初始化状态并完成倾斜和偏航对准。 当倾斜和偏航对齐完成后，EKF 可以转换到其他操作模式，从而可以使用其他传感器数据：

### GPS

如果满足以下条件，GPS 测量将用于位置和速度：

* 通过设置 [EKF2_AID_MASK](../advanced/parameter_reference.md#EKF2_AID_MASK) 参数启用 GPS 使用。
* GPS 质量检查已通过。 这些检查由 [EKF2_GPS_CHECK](../advanced/parameter_reference.md#EKF2_GPS_CHECK) 和 `EKF2\_REQ\_\*` 参数控制。 
* 通过设置 [EKF2_HGT_MODE](../advanced/parameter_reference.md#EKF2_HGT_MODE) 参数，EKF 可以直接使用 GPS 高度。

### 测距仪

单个状态滤波器使用距离探测器到地面的距离来估计地形相对于高度基准的垂直位置。

如果在可用作零高度基准面的平面上操作，则 EKF 也可以直接使用测距仪数据，通过将 [EKF2_HGT_MODE](../advanced/parameter_reference.md#EKF2_HGT_MODE) 参数设置为 2 来估算高度。

### 空速：

通过将 <E>EKF2_ARSP_THR</a> 设置为正值，等效空速\（EAS\）数据可用于估计风速并减少 GPS 丢失时的漂移。 当空速超过由 [EKF2_ARSP_THR](../advanced/parameter_reference.md#EKF2_ARSP_THR) 的正值设定的阈值并且飞机类型不是旋翼时，将使用空速数据。

### 合成 Sideslip

固定翼平台可以利用零假设侧滑观测来改善风速估计，并且还可以在没有空速传感器的情况下进行风速估算。 通过将 [EKF2_FUSE_BETA](../advanced/parameter_reference.md#EKF2_FUSE_BETA) 参数设置为 1 来启用此功能。

### 拖动特定力量

多转子平台可以利用沿 X 和 Y 体轴的空速和阻力之间的关系来估计风速的北/东分量。 通过将 [EKF2_AID_MASK](../advanced/parameter_reference.md#EKF2_AID_MASK) 参数中的位位置5设置为 true 来启用此功能。 沿 X 和 Y 体轴的空速和特定力\（IMU 加速度\）之间的关系由 [EKF2_BCOEF_X](../advanced/parameter_reference.md#EKF2_BCOEF_X) 和 [EKF2_BCOEF_Y](../advanced/parameter_reference.md#EKF2_BCOEF_Y) 参数控制，这些参数设定了飞行中的弹道系数。 分别是 X 和 Y 方向。 特定力观察噪声的量由 [EKF2_DRAG_NOISE](../advanced/parameter_reference.md#EKF2_DRAG_NOISE) 参数设定。

### 光流

如果满足以下条件，将使用光流数据：

* 有效的测距仪数据可用。
* [EKF2_AID_MASK](../advanced/parameter_reference.md#EKF2_AID_MASK) 参数中的位位置 1 为真。
* 流量传感器返回的质量度量值大于 [EKF2_OF_QMIN](../advanced/parameter_reference.md#EKF2_OF_QMIN) 参数设置的最低要求。

### 外部视觉系统

来自外部视觉系统的位置和姿势测量，例如， Vicon，可以使用：

* 如果 [EKF2_AID_MASK](../advanced/parameter_reference.md#EKF2_AID_MASK) 参数中的位位置 3 为真，则将使用外部视觉系统水平位置数据。
* 如果 [EKF2_HGT_MODE](../advanced/parameter_reference.md#EKF2_HGT_MODE) 参数设置为 3，将使用外部视觉系统垂直位置数据。
* 如果 [EKF2_AID_MASK](../advanced/parameter_reference.md#EKF2_AID_MASK) 参数中的位位置 4 为真，则外部视觉系统姿势数据将用于偏航估计。

## 如何使用'ecl'库 EKF？

将 [SYS_MC_EST_GROUP](../advanced/parameter_reference.md#SYS_MC_EST_GROUP) 参数设置为 2 以使用 ecl EKF。

## Ecl EKF 优于其他估算器的优点和缺点是什么？

与所有估算器一样，大部分性能来自调整以匹配传感器特性。 Tuning 是准确性和鲁棒性之间的折衷，虽然我们试图提供满足大多数用户需求的调优，但是应用程序需要调整更改。

出于这个原因，没有相对于 legacy\_estimator\_q + local\_position\_estimator 的传统组合的准确性声明，并且估计器的最佳选择将取决于应用和调整。

### 缺点

* Ecl EKF 是一种复杂的算法，需要很好地理解扩展卡尔曼滤波器理论及其在导航问题中的应用才能成功调整。 因此，未达到良好结果的用户更难以知道要改变什么。
* Ecl EKF 使用更多 RAM 和闪存空间。
* Ecl EKF 使用更多的日志空间。
* Ecl EKF 的飞行时间较短。

### 优势

* Ecl EKF 能够以数学上一致的方式融合来自具有不同时间延迟和数据速率的传感器的数据，一旦正确设置时间延迟参数，就可以提高动态操作期间的准确性。
* Ecl EKF 能够融合各种不同的传感器类型。
* Ecl EKF 检测并报告传感器数据中统计上显着的不一致性，帮助诊断传感器错误。
* 对于固定机翼操作，ecl EKF 使用或不使用空速传感器估算风速，并且能够将估计的风与空速测量和侧滑假设结合使用，以延长 GPS 在飞行中丢失时的航位推算时间。
* Ecl EKF估计3轴加速度计偏差，这提高了尾部和其他在飞行阶段之间经历大的姿态变化的无人机的精度。
* 联合架构\（组合姿态和位置/速度估计\）意味着姿态估计受益于所有传感器测量。 如果调整正确，这应该提供改善态度估计的潜力。 

## 如何检查 EKF 性能？

EKF 输出，状态和状态数据发布到许多 uORB 主题，这些主题在飞行期间记录到 SD 卡。 以下指南假定已使用 .ulog 文件格式记录数据。 要使用 .ulog 格式，请将 SYS\_LOGGER 参数设置为 1。

可以使用 [PX4 pyulog library](https://github.com/PX4/pyulog) 在 python 中解析 .ulog 格式数据。

大多数 EKF 数据位于记录到 .ulog 文件的 [ekf2\_inovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg) 和 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg)uORB 消息中。

可以在 [here](https://github.com/PX4/Firmware/blob/master/Tools/ecl_ekf/process_logdata_ekf.py) 找到自动生成分析图和元数据的 python 脚本。 要使用此脚本文件，请 cd 到 `Tools/ecl_ekf`directory and enter`python process_logdata_ekf.py&lt;log_file.ulg&gt;`。 这将性能元数据保存在名为 `&lt;log_file&gt;.mdat.csv` 的 csv 文件中，并绘制在名为 `&lt;log_file&gt;.pdf` 的 pdf 文件中。

可以使用 [batch\_process\_logdata\_ekf.py](https://github.com/PX4/Firmware/blob/master/Tools/ecl_ekf/batch_process_logdata_ekf.py) 脚本分析目录中的多个日志文件。 完成此操作后，可以处理性能元数据文件，以使用 [batch\_process\_metadata\_ekf.py](https://github.com/PX4/Firmware/blob/master/Tools/ecl_ekf/batch_process_metadata_ekf.py) 脚本对日志总数中的估计器性能进行统计评估。

### 输出数据

* 姿态输出数据可在 [vehicle\_attitude](https://github.com/PX4/Firmware/blob/master/msg/vehicle_attitude.msg) 消息中找到。
* Ocal 位置输出数据可在 [vehicle\_local\_position](https://github.com/PX4/Firmware/blob/master/msg/vehicle_local_position.msg) 消息中找到。
* Global \（WGS-84\）输出数据位于 [vehicle\_global\_position](https://github.com/PX4/Firmware/blob/master/msg/vehicle_global_position.msg) 消息中。
* 风速输出数据可在 [wind\_estimate](https://github.com/PX4/Firmware/blob/master/msg/wind_estimate.msg) 消息中找到。

### 状态

请参阅 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) 中的 states\[32\]。 状态 \[32\] 的索引映射如下：

* \[0 ... 3\] 四元数
* \[4 ... 6\] Velocity NED \(m/s\)
* \[7 ... 9\] Position NED \(m\)
* \[10 ... 12\] IMU delta angle bias XYZ \(rad\)
* \[13 ... 15\] IMU delta velocity bias XYZ \(m/s\)
* \[16 ... 18\] Earth magnetic field NED \(gauss\)
* \[19 ... 21\] Body magnetic field XYZ \(gauss\)
* \[22 ... 23\] Wind velocity NE \(m/s\)
* \[24 ... 32\] Not Used

### State Variances

Refer to covariances\[28\] in [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg). The index map for covariances\[28\] is as follows:

* \[0 ... 3\] Quaternions
* \[4 ... 6\] Velocity NED \(m/s\)^2
* \[7 ... 9\] Position NED \(m^2\)
* \[10 ... 12\] IMU delta angle bias XYZ \(rad^2\)
* \[13 ... 15\] IMU delta velocity bias XYZ \(m/s\)^2
* \[16 ... 18\] Earth magnetic field NED \(gauss^2\)
* \[19 ... 21\] Body magnetic field XYZ \(gauss^2\)
* \[22 ... 23\] Wind velocity NE \(m/s\)^2
* \[24 ... 28\] Not Used

### 新息（ t 时刻的实际量测值 与 t-1 时刻对 t 时刻量测的估计值与之间的差）

* Magnetometer XYZ \(gauss\) : Refer to mag\_innov\[3\] in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Yaw angle \(rad\) : Refer to heading\_innov in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Velocity and position innovations : Refer to vel\_pos\_innov\[6\] in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg). The index map for vel\_pos\_innov\[6\] is as follows: 
  * \[0 ... 2\] Velocity NED \(m/s\)
  * \[3 ... 5\] Position NED \(m\)
* True Airspeed \(m/s\) : Refer to airspeed\_innov in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Synthetic sideslip \(rad\) : Refer to beta\_innov in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Optical flow XY \(rad/sec\) : Refer to flow\_innov in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Height above ground \(m\) : Refer to hagl\_innov in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).

### Observation Innovation Variances

* Magnetometer XYZ \(gauss^2\) : Refer to mag\_innov\_var\[3\] in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Yaw angle \(rad^2\) : Refer to heading\_innov\_var in the ekf2\_innovations message.
* Velocity and position innovations : Refer to vel\_pos\_innov\_var\[6\] in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg). The index map for vel\_pos\_innov\_var\[6\] is as follows: 
  * \[0 ... 2\] Velocity NED \(m/s\)^2
  * \[3 ... 5\] Position NED \(m^2\)
* True Airspeed \(m/s\)^2 : Refer to airspeed\_innov\_var in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Synthetic sideslip \(rad^2\) : Refer to beta\_innov\_var in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Optical flow XY \(rad/sec\)^2 : Refer to flow\_innov\_var in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Height above ground \(m^2\) : Refer to hagl\_innov\_var in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).

### Output Complementary Filter

The output complementary filter is used to propagate states forward from the fusion time horizon to current time. To check the magnitude of the angular, velocity and position tracking errors measured at the fusion time horizon, refer to output\_tracking\_error\[3\] in the ekf2\_innovations message. The index map is as follows:

* \[0\] Angular tracking error magnitude \(rad\)
* \[1\] Velocity tracking error magnitude \(m/s\). The velocity tracking time constant can be adjusted using the [EKF2_TAU_VEL](../advanced/parameter_reference.md#EKF2_TAU_VEL) parameter. Reducing this parameter reduces steady state errors but increases the amount of observation noise on the NED velocity outputs.
* \[2\] Position tracking error magnitude \(m\). The position tracking time constant can be adjusted using the [EKF2_TAU_POS](../advanced/parameter_reference.md#EKF2_TAU_POS) parameter. Reducing this parameter reduces steady state errors but increases the amount of observation noise on the NED position outputs.

### EKF Errors

The EKF contains internal error checking for badly conditioned state and covariance updates. Refer to the filter\_fault\_flags in [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).

### Observation Errors

There are two categories of observation faults:

* Loss of data. An example of this is a range finder failing to provide a return.
* The innovation, which is the difference between the state prediction and sensor observation is excessive. An example of this is excessive vibration causing a large vertical position error, resulting in the barometer height measurement being rejected.

Both of these can result in observation data being rejected for long enough to cause the EKF to attempt a reset of the states using the sensor observations. All observations have a statistical confidence checks applied to the innovations. The number of standard deviations for the check are controlled by the EKF2\_\*\_GATE parameter for each observation type.

Test levels are available in [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) as follows:

* mag\_test\_ratio : ratio of the largest magnetometer innovation component to the innovation test limit
* vel\_test\_ratio : ratio of the largest velocity innovation component to the innovation test limit
* pos\_test\_ratio : ratio of the largest horizontal position innovation component to the innovation test limit
* hgt\_test\_ratio : ratio of the vertical position innovation to the innovation test limit
* tas\_test\_ratio : ratio of the true airspeed innovation to the innovation test limit
* hagl\_test\_ratio : ratio of the height above ground innovation to the innovation test limit

For a binary pass/fail summary for each sensor, refer to innovation\_check\_flags in [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).

### GPS Quality Checks

The EKF applies a number of GPS quality checks before commencing GPS aiding. These checks are controlled by the [EKF2_GPS_CHECK](../advanced/parameter_reference.md#EKF2_GPS_CHECK) and `EKF2_REQ_*` parameters. The pass/fail status for these checks is logged in the [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).gps\_check\_fail\_flags message. This integer will be zero when all required GPS checks have passed. If the EKF is not commencing GPS alignment, check the value of the integer against the bitmask definition gps\_check\_fail\_flags in [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).

### EKF Numerical Errors

The EKF uses single precision floating point operations for all of its computations and first order approximations for derivation of the covariance prediction and update equations in order to reduce processing requirements. This means that it is possible when re-tuning the EKF to encounter conditions where the covariance matrix operations become badly conditioned enough to cause divergence or significant errors in the state estimates.

To prevent this, every covariance and state update step contains the following error detection and correction steps:

* If the innovation variance is less than the observation variance \(this requires a negative state variance which is impossible\) or the covariance update will produce a negative variance for any of the states, then: 
  * The state and covariance update is skipped
  * The corresponding rows and columns in the covariance matrix are reset
  * The failure is recorded in the [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) filter\_fault\_flags message
* State variances \(diagonals in the covariance matrix\) are constrained to be non-negative.
* An upper limit is applied to state variances.
* Symmetry is forced on the covariance matrix.

After re-tuning the filter, particularly re-tuning that involve reducing the noise variables, the value of estimator\_status.gps\_check\_fail\_flags should be checked to ensure that it remains zero.

## What should I do if the height estimate is diverging?

The most common cause of EKF height diverging away from GPS and altimeter measurements during flight is clipping and/or aliasing of the IMU measurements caused by vibration. If this is occurring, then the following signs should be evident in the data

* [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).vel\_pos\_innov\[2\] and [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).vel\_pos\_innov\[5\] will both have the same sign.
* [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).hgt\_test\_ratio will be greater than 1.0

The recommended first step is to ensure that the autopilot is isolated from the airframe using an effective isolation mounting system. An isolation mount has 6 degrees of freedom, and therefore 6 resonant frequencies. As a general rule, the 6 resonant frequencies of the autopilot on the isolation mount should be above 25Hz to avoid interaction with the autopilot dynamics and below the frequency of the motors.

An isolation mount can make vibration worse if the resonant frequencies coincide with motor or propeller blade passage frequencies.

The EKF can be made more resistant to vibration induced height divergence by making the following parameter changes:

* Double the value of the innovation gate for the primary height sensor. If using barometric height this is [EKF2_BARO_GATE](../advanced/parameter_reference.md#EKF2_BARO_GATE).
* Increase the value of [EKF2_ACC_NOISE](../advanced/parameter_reference.md#EKF2_ACC_NOISE) to 0.5 initially. If divergence is still occurring, increase in further increments of 0.1 but do not go above 1.0

Note that the effect of these changes will make the EKF more sensitive to errors in GPS vertical velocity and barometric pressure.

## What should I do if the position estimate is diverging?

The most common causes of position divergence are:

* High vibration levels. 
  * Fix by improving mechanical isolation of the autopilot.
  * Increasing the value of [EKF2_ACC_NOISE](../advanced/parameter_reference.md#EKF2_ACC_NOISE) and [EKF2_GYR_NOISE](../advanced/parameter_reference.md#EKF2_GYR_NOISE) can help, but does make the EKF more vulnerable to GPS glitches.
* Large gyro bias offsets. 
  * Fix by re-calibrating the gyro. Check for excessive temperature sensitivity \(&gt; 3 deg/sec bias change during warm-up from a cold start and replace the sensor if affected of insulate to to slow the rate of temperature change.
* Bad yaw alignment 
  * Check the magnetometer calibration and alignment.
  * Check the heading shown QGC is within within 15 deg truth
* Poor GPS accuracy 
  * Check for interference
  * Improve separation and shielding
  * Check flying location for GPS signal obstructions and reflectors \(nearby tall buildings\)
* Loss of GPS

Determining which of these is the primary cause requires a methodical approach to analysis of the EKF log data:

* Plot the velocity innovation test ratio - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vel\_test\_ratio
* Plot the horizontal position innovation test ratio - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).pos\_test\_ratio
* Plot the height innovation test ratio - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).hgt\_test\_ratio
* Plot the magnetometer innovation test ratio - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).mag\_test\_ratio
* Plot the GPS receiver reported speed accuracy - [vehicle\_gps\_position](https://github.com/PX4/Firmware/blob/master/msg/vehicle_gps_position.msg).s\_variance\_m\_s
* Plot the IMU delta angle state estimates - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).states\[10\], states\[11\] and states\[12\]
* Plot the EKF internal high frequency vibration metrics: 
  * Delta angle coning vibration - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vibe\[0\]
  * High frequency delta angle vibration - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vibe\[1\]
  * High frequency delta velocity vibration - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vibe\[2\]

During normal operation, all the test ratios should remain below 0.5 with only occasional spikes above this as shown in the example below from a successful flight:

![Position, Velocity, Height and Magnetometer Test Ratios](../../assets/ecl/test_ratios_-_successful.png)

The following plot shows the EKF vibration metrics for a multirotor with good isolation. The landing shock and the increased vibration during takeoff and landing can be seen. Insufficient data has been gathered with these metrics to provide specific advice on maximum thresholds.

![](../../assets/ecl/vibration_metrics_-_successful.png)

The above vibration metrics are of limited value as the presence of vibration at a frequency close to the IMU sampling frequency \(1 kHz for most boards\) will cause offsets to appear in the data that do not show up in the high frequency vibration metrics. The only way to detect aliasing errors is in their effect on inertial navigation accuracy and the rise in innovation levels.

In addition to generating large position and velocity test ratios of &gt; 1.0, the different error mechanisms affect the other test ratios in different ways:

### Determination of Excessive Vibration

High vibration levels normally affect vertical position and velocity innovations as well as the horizontal components. Magnetometer test levels are only affected to a small extent.

\(insert example plots showing bad vibration here\)

### Determination of Excessive Gyro Bias

Large gyro bias offsets are normally characterised by a change in the value of delta angle bias greater than 5E-4 during flight \(equivalent to ~3 deg/sec\) and can also cause a large increase in the magnetometer test ratio if the yaw axis is affected. Height is normally unaffected other than extreme cases. Switch on bias value of up to 5 deg/sec can be tolerated provided the filter is given time time settle before flying . Pre-flight checks performed by the commander should prevent arming if the position is diverging.

\(insert example plots showing bad gyro bias here\)

### Determination of Poor Yaw Accuracy

Bad yaw alignment causes a velocity test ratio that increases rapidly when the vehicle starts moving due inconsistency in the direction of velocity calculated by the inertial nav and the GPS measurement. Magnetometer innovations are slightly affected. Height is normally unaffected.

\(insert example plots showing bad yaw alignment here\)

### Determination of Poor GPS Accuracy

Poor GPS accuracy is normally accompanied by a rise in the reported velocity error of the receiver in conjunction with a rise in innovations. Transient errors due to multipath, obscuration and interference are more common causes. Here is an example of a temporary loss of GPS accuracy where the multi-rotor started drifting away from its loiter location and had to be corrected using the sticks. The rise in [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vel\_test\_ratio to greater than 1 indicates the GPs velocity was inconsistent with other measurements and has been rejected.

![](../../assets/ecl/gps_glitch_-_test_ratios.png)

This is accompanied with rise in the GPS receivers reported velocity accuracy which indicates that it was likely a GPS error.

![](../../assets/ecl/gps_glitch_-_reported_receiver_accuracy.png)

If we also look at the GPS horizontal velocity innovations and innovation variances, we can see the large spike in North velocity innovation that accompanies this GPS 'glitch' event.

![](../../assets/ecl/gps_glitch_-_velocity_innovations.png)

### Determination of GPS Data Loss

Loss of GPS data will be shown by the velocity and position innovation test ratios 'flat-lining'. If this occurs, check the other GPS status data in `vehicle_gps_position` for further information.

The following plot shows the NED GPS velocity innovations `ekf2_innovations_0.vel_pos_innov[0 ... 2]`, the GPS NE position innovations `ekf2_innovations_0.vel_pos_innov[3 ... 4]` and the Baro vertical position innovation `ekf2_innovations_0.vel_pos_innov[5]` generated from a simulated VTOL flight using SITL Gazebo.

The simulated GPS was made to lose lock at 73 seconds. Note the NED velocity innovations and NE position innovations 'flat-line' after GPs is lost. Note that after 10 seconds without GPS data, the EKF reverts back to a static position mode using the last known position and the NE position innovations start to change again.

![GPS Data Loss - in SITL](../../assets/ecl/gps_data_loss_-_velocity_innovations.png)