# 使用 ecl EKF

本教程旨在解答一些关于ECL EKF算法使用的常见问题。

## 什么是 ecl EKF?

ECL(Estimation and Control Library，估计与控制库）使用EKF(Extended Kalman Filter，扩展卡尔曼滤波器)算法来处理传感器测量信息并为下面的状态提供估计：

* 四元数，定义为从地球NED(北东地)坐标系到机体坐标系X,Y,Z的旋转四元数
* 速度，关于IMU - 北，东，地 (m/s)
* 位置，关于IMU - 北，东，地 (m)
* 角速度偏移估计，关于IMU - X,Y,Z (rad)
* 速度偏移估计，关于IMU - X,Y,Z (m/s)
* 地磁分量 - 北，东，地 (gauss)
* 磁偏量，关于飞行器本身 - X,Y,Z (gauss)
* 风速 - 北，东 (m/s)

EKF在一个延迟的`fusion time horizon`上运行，以允许传感器在每次测量时相对于IMU存在不同的时间延迟。 每个传感器的数据都是FIFO缓存的并由EKF从缓存中检索，得以在正确的时候使用。每个传感器的延迟补偿由参数`EKF2_*_DELAY`控制。

互补滤波器用于利用缓存好的IMU数据将状态从`fusion time horizon`向前传播到当前时间。该滤波器的时间常数由参数`EKF2_TAU_VEL `和`EKF2_TAU_POS`控制。

> **注意：**`fusion time horizon`延迟和缓冲区的长度由参数`EKF2_*_DELAY`的最大值确定。如果未使用某传感器，建议将其时间延迟设置为零。降低`fusion time horizon`延迟会减少互补滤波器中用于将状态向前传播到当前时间的误差

调整位置和速度状态以消除IMU和机体坐标系之间由于安装误差所产生的偏移，避免其输出到控制回路。 IMU相对于机体坐标系的位置由参数`EKF2_IMU_POS_X，Y，Z`设置。

EKF仅使用IMU的数据进行状态预测。IMU的数据不会用作EKF推导过程中的量测值。协方差预测、状态更新以及协方差更新的代数方程都是使用Matlab符号工具箱导出的，可以在这里找到：[Matlab Symbolic Derivation](https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m)

## ecl EKF使用何种传感器测量值？

根据传感器测量值的不同组合，EKF具有不同的操作模式。在启动时，滤波器会检查传感器的最小可行组合，并在初始倾斜、偏航以及高度对准完成后进入一个提供旋转、垂直速度、垂直位置、IMU角增量误差和IMU速度增量误差估计的模式。

此模式需要有传感器的数据，例如偏航数据源（由磁力计或者外部视觉设备提供）和高度数据源。所有的EKF操作模式都需要这个最小的数据集。然后可以使用其他传感器数据来估计附加的状态。

### 惯性测量单元（IMU）

* 惯性测量单元的三轴位置固定，用于采集单位角增量和速度增量数据，最低采样频率为100Hz。

> 注意：在EKF使用IMU角增量数据之前应对其进行校正修正。

### 磁力计

三轴磁力计（或者是外部视觉系统）的最低采样率为5Hz。磁力计数据可以以两种方式使用：

* 使用倾斜估计和磁偏角将磁力计测量值转换为偏航角。该偏航角被用作EKF的观测量。这种方法不太准确并且没有考虑机体坐标系的磁场偏移，然而其对于磁异常和大的陀螺仪初始偏移更加鲁棒。它是飞行器刚启动还停留在地面上时使用的默认方法。
* 磁力计的XYZ三轴读数用作单独测观测值。这种方法更准确，并且允许机体坐标系的偏移。但假定地磁场环境只是缓慢变化，这种方法在存在巨大的外部磁异常时性能较差。这是当飞行器在空中并爬升超过1.5米高度时的默认方法。

用于选择模式的逻辑由EKF2_MAG_TYPE参数设置。

### 高度

高度的数据源——GPS，气压，测距仪或外部视觉（最低采样频率5Hz）。注意：高度数据的主要来源由EKF2\_HGT\_MODE参数控制。

如果这些测量不存在，EKF将不会启动。当检测到这些测量时，EKF将初始化状态并完成倾斜和偏航对准。当倾斜和偏航对准完成时，EKF可以转换到其他的操作模式，使得能够使用附加的传感器数据：

### GPS

如果满足以下条件，则GPS测量将用于位置和速度：

* 通过设置EKF2\_AID\_MASK参数使能GPS的使用
* GPS通过质量检查。这些检查由EKF2_GPS_CHECK和 EKF2_REQ<>参数控制
* 通过EKF2_HGT_MODE参数的设置，EKF可以直接使用GPS高度。

### 测距仪

测距仪到地面的距离由单个状态滤波器使用，用于估计地形相对于高度基准的垂直位置。

如果在可用作零高度基准的平坦表面上操作，测距仪数据也可以直接由EKF使用，以通过将EKF2_HGT_MODE参数设置为2来估计高度。

### 空速

通过将EKF2_ARSP_THR设置为正值，等效空速（EAS）数据可用于估计风速并减少GPS丢失时的漂移。 当超过由EKF2_ARSP_THR设置的阈值并且飞行器不是旋翼时，将使用空速数据。

### Synthetic Sideslip

固定翼平台可以利用假设的边缘观测零点来改进风速估计，并且还能够在没有空速传感器的情况下实现风速估计。可以通过将EKF2_FUSE_BETA参数设置为1来启用此项。

### 光流

如果满足以下条件，将使用光流数据：

* 有效的测距仪数据可用。
* EKF2_AID_MASK参数中第1位为真。
* 光流传感器返回的质量度量大于由EKF2_OF_QMIN参数设置的最小要求

### 外部视觉系统

位置和姿态可以使用外部视觉系统（例如Vicon）进行测量：

* 如果EKF2_AID_MASK参数中的第3位为真，则将使用外部视觉系统水平位置数据。
* 如果EKF2_HGT_MODE参数设置为3，则将使用外部视觉系统垂直位置数据。
* 如果EKF2_AID_MASK参数中的第4位为真，则外部视觉系统的姿态数据将用于偏航估计。

## 如何使用ecl EKF?

将SYS\_MC\_EST\_GROUP参数设置为2可使用ecl EKF.

## 相比其他估计器ecl EKF有什么优缺点?

与所有估计器一样，大多数性能需要进行调参以匹配传感器特性。调参是在精度和鲁棒性之间的折衷，尽管我们试图提供满足大多数用户需求的参数，但是仍将存在需要进行参数调节的应用。

出于这个原因，相对于attitude\_estimator\_q + local\_position\_estimator的传统估计器组合，ecl EKF没有精度方面的要求(claim)，估计器的最优选取将取决于应用以及参数调节。

### 缺点

* ecl EKF是一个复杂的算法，需要很好地了解扩展卡尔曼滤波理论及其在导航问题中的应用才能更好的进行参数调节。因此，对于没有获得良好效果地用户来说，知道要改变什么则更为困难。


* ecl EKF使用更多的RAM和闪存空间
* ecl EKF使用更多的日志空间
* ecl EKF具有较少的飞行时间

### 优点

* ecl EKF能够以数学上一致的方式融合来自具有不同时间延迟和采样频率的传感器的数据，在正确地设置了时间延迟参数的情况下，该算法将提高动态操纵精度。
* ecl EKF能够融合大范围的不同传感器类型。
* ecl EKF检测并报告传感器数据统计中显著的不一致情况，可协助诊断传感器错误。
* 对于固定翼操作，无论是否使用空速传感器，ecl EKF都能够用于估计风速，并且能够使用估计的风速结合空速测量以及侧滑假设来延长可用的航位推算时间(如果在飞行中丢失GPS信号)。
* ecl EKF估计3轴加速度计偏差，这提高了立式起落飞行器(tailsitters)和其他飞行器在飞行阶段经历大的姿态变化时的精度。
* The federated architecture \(combined attitude and position/velocity estimation\) means that attitude estimation benefits from all sensor measurements. This should provide the potential for improved attitude estimation if tuned correctly.
* 联合架构（将姿态和位置/速度估计结合起来）意味着姿态估计会受益于所有传感器测量。 如果适当调节，这将提供改进姿态估计的潜力。

## 我需要如何检查EKF的执行效果？

在飞行过程中，EKF 的输出（outputs）, 状态（states） 以及状态数据（status data）会发布到许多uORB话题（Topic）中，这些话题会存储到SD卡中。
下面的介绍假设数据已经以.ulog文件的形式被记录，为了使用.ulog格式，需要将 SYS\_LOGGER 参数置1。

.ulog格式文件可以在python中使用[PX4 pyulog library](https://github.com/PX4/pyulog)来进行分析。

大部分 EKF 数据都在.ulog文件中的 [ekf2_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg)和 [estimator_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) 这两个uORB消息里面。

### Output Data（输出数据）

* Attitude output data is found in the [vehicle_attitude](https://github.com/PX4/Firmware/blob/master/msg/vehicle_attitude.msg) message.（姿态数据）
* Local position output data is found in the [vehicle_local_position](https://github.com/PX4/Firmware/blob/master/msg/vehicle_local_position.msg)message.（位置数据）
* Control loop feedback data is found in the the [control_state](https://github.com/PX4/Firmware/blob/master/msg/control_state.msg) message.（控制环反馈数据）
* Global \(WGS-84\) output data is found in the [vehicle_global_position](https://github.com/PX4/Firmware/blob/master/msg/vehicle_global_position.msg) message.（全球位置数据）
* Wind velocity output data is found in the [wind_estimate](https://github.com/PX4/Firmware/blob/master/msg/wind_estimate.msg) message.（风速数据）

### States（状态）

查看[estimator_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg)中的states\[32\]。状态的索引如下：

* \[0 ... 3\] Quaternions
* \[4 ... 6\] Velocity NED \(m/s\)
* \[7 ... 9\] Position NED \(m\)
* \[10 ... 12\] IMU delta angle bias XYZ \(rad\)
* \[13 ... 15\] IMU delta velocity bias XYZ \(m/s\)
* \[16 ... 18\] Earth magnetic field NED \(gauss\)
* \[19 ... 21\] Body magnetic field XYZ \(gauss\)
* \[22 ... 23\] Wind velocity NE \(m/s\)
* \[24 ... 32\] Not Used

### State Variances（状态的变化量）

查看[estimator_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg)中的covariances\[28\] 。变量的索引如下：

* \[0 ... 3\] Quaternions
* \[4 ... 6\] Velocity NED \(m/s\)^2
* \[7 ... 9\] Position NED \(m^2\)
* \[10 ... 12\] IMU delta angle bias XYZ \(rad^2\)
* \[13 ... 15\] IMU delta velocity bias XYZ \(m/s\)^2
* \[16 ... 18\] Earth magnetic field NED \(gauss^2\)
* \[19 ... 21\] Body magnetic field XYZ \(gauss^2\)
* \[22 ... 23\] Wind velocity NE \(m/s\)^2
* \[24 ... 28\] Not Used

### Observation Innovations

* Magnetometer XYZ \(gauss\) : Refer to mag\_innov\[3\] in [ekf2_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Yaw angle \(rad\) : Refer to heading\_innov in [ekf2_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Velocity and position innovations : Refer to vel\_pos\_innov\[6\] in [ekf2_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg). The index map for vel\_pos\_innov\[6\] is as follows:
* \[0 ... 2\] Velocity NED \(m/s\)
* \[3 ... 5\] Position NED \(m\)
* True Airspeed \(m/s\) : Refer to airspeed\_innov in [ekf2_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Synthetic sideslip \(rad\) : Refer to beta\_innov in [ekf2_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Optical flow XY \(rad/sec\) : Refer to flow\_innov in [ekf2_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Height above ground \(m\) : Refer to hagl\_innov in [ekf2_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).

### Observation Innovation Variances

* Magnetometer XYZ \(gauss^2\) : Refer to mag\_innov\_var\[3\] in [ekf2_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Yaw angle \(rad^2\) : Refer to heading\_innov\_var in the ekf2\_innovations message.
* Velocity and position innovations : Refer to vel\_pos\_innov\_var\[6\] in [ekf2_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg). The index map for vel\_pos\_innov\_var\[6\] is as follows:
* \[0 ... 2\] Velocity NED \(m/s\)^2
* \[3 ... 5\] Position NED \(m^2\)
* True Airspeed \(m/s\)^2 : Refer to airspeed\_innov\_var in [ekf2_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Synthetic sideslip \(rad^2\) : Refer to beta\_innov\_var in [ekf2_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Optical flow XY \(rad/sec\)^2 : Refer to flow\_innov\_var in [ekf2_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Height above ground \(m^2\) : Refer to hagl\_innov\_var in [ekf2_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).

### Output Complementary Filter（输出互补滤波器）

The output complementary filter is used to propagate states forward from the fusion time horizon to current time. To check the magnitude of the angular, velocity and position tracking errors measured at the fusion time horizon, refer to output\_tracking\_error\[3\] in the ekf2\_innovations message. The index map is as follows:

* \[0\] Angular tracking error magnitude \(rad\)
* \[1\] Velocity tracking error magntiude \(m/s\). The velocity tracking time constant can be adjusted using the EKF2\_TAU\_VEL parameter. Reducing this parameter reduces steady state errors but increases the amount of observation noise on the NED velocity outputs.
* \[2\] Position tracking error magntiude \(m\). The position tracking time constant can be adjusted using the EKF2\_TAU\_POS parameter. Reducing this parameter reduces steady state errors but increases the amount of observation noise on the NED position outputs.

### EKF Errors

The EKF constains internal error checking for badly conditioned state and covariance updates. Refer to the filter\_fault\_flags in [estimator_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).

### Observation Errors

There are two categories of observation faults:

* Loss of data. An example of this is a range finder failing to provide a return.
* The innovation, which is the difference between the state prediction and sensor observation is excessive. An example of this is excessive vibration causing a large vertical position error, resulting in the barometer height measurement being rejected.

Both of these can result in observation data being rejected for long enough to cause the EKF to attempt a reset of the states using the sensor observations. All observations have a statistical confidence check applied to the innovations. The number of standard deviations for the check are controlled by the EKF2\_&lt;&gt;\_GATE parameter for each observation type.

Test levels are  available in [estimator_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) as follows:

* mag\_test\_ratio : ratio of the largest magnetometer innovation component to the innovation test limit
* vel\_test\_ratio : ratio of the largest velocity innovation component to the innovation test limit
* pos\_test\_ratio : ratio of the largest horizontal position innovation component to the innovation test limit
* hgt\_test\_ratio : ratio of the vertical position innovation to the innovation test limit
* tas\_test\_ratio : ratio of the true airspeed innovation to the innovation test limit
* hagl\_test\_ratio : ratio of the height above ground innovation to the innovation test limit

For a binary pass/fail summary for each sensor, refer to innovation\_check\_flags in [estimator_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).

### GPS Quality Checks

The EKF applies a number of GPS quality checks before commencing GPS aiding. These checks are controlled by the EKF2\_GPS\_CHECK and EKF2\_REQ&lt;&gt; parameters. The pass/fail status for these checks is logged in the [estimator_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).gps\_check\_fail\_flags message. This integer will be zero when all required GPS checks have passed. If the EKF is not commencing GPS alignment, check the value of the integer against the bitmask definition gps\check\_fail\_flags in  [estimator_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).

### EKF Numerical Errors

The EKF uses single precision floating point operations for all of its computations and first order approximations for derivation of the covariance prediction and update equations in order to reduce processing requirements. This means that it is possible when re-tuning the EKF to encounter conditions where the covariance matrix operations become badly conditioned enough to cause divergence or significant errors in the state estimates.

To prevent this, every covariance and state update step contains the following error detection and correction steps:

* If the innovation variance is less than the observation variance \(this requires a negative state variance which is impossible\) or the covariance update will produce a negative variance for any of the states, then:
  * The state and covariance update is skipped
  * The corresponding rows and columns in the covariance matrix are reset
  * The failure is recorded in the [estimator_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) filter\_fault\_flags messaage


* State variances \(diagonals in the covariance matrix\) are constrained to be non-negative.
* An upper limit is applied to state variances.
* Symmetry is forced on the covariance matrix.

After re-tuning the filter, particularly re-tuning that involve reducing the noise variables,  the value of estimator\_status.gps\_check\_fail\_flags should be checked to ensure that it remains zero.

## What should I do if the height estimate is diverging?

The most common cause of EKF height diverging away from GPS and altimeter measurements during flight is clipping and/or aliasing of the IMU measurements caused by vibration. If this is occurring, then the following signs should be evident in the data

* [ekf2_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).vel\_pos\_innov\[3\] and  [ekf2_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).vel\_pos\_innov\[5\] will both have the same sign.
* [estimator_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).hgt\_test\_ratio will be greater than 1.0


The recommended first step is to  esnure that the autopilot is isolated from the airframe using an effective isolatoin mounting system. An isolaton mount has 6 degrees of freedom, and therefore 6 resonant frequencies. As a general rule, the 6 resonant frequencies of the autopilot on the isolation mount should be above 25Hz to avoid interaction with the autopilot dynamics and below the frequency of the motors.

An isolation mount can make vibration worse if the resonant frequncies coincide with motor or propeller blade passage frequencies.

The EKF can be made more resistant to vibration induced height divergence by making the following parameter changes:

* Double the value of the innovation gate for the primary height sensor. If using barometeric height this is EK2\_EKF2\_BARO\_GATE.
* Increase the value of EKF2\_ACC\_NOISE to 0.5 initially. If divergence is still occurring,   increase in further increments of 0.1 but do not go above 1.0

Note that the effect of these changes will make the EKF more sensitive to errors in GPS vertical velocity and barometric pressure.

## What should I do if the position estimate is diverging?

The most common causes of position divergence are:

* High vibration levels.
  * Fix by improving mechanical isolation of the autopilot.
  * Increasing the value of EKF2\_ACC\_NOISE and EKF2\_GYR\_NOISE can help, but does make the EKF more vulnerable to GPS glitches.

* Large gyro bias offsets.
  * Fix by re-calibrating the gyro. Check for excessive temperature sensitivity \(&gt; 3 deg/sec bias change during warm-up from a cold start and replace the sensor if affected of insulate to to slow the rate of temeprature change.

* Bad yaw alignment
  * Check the magntometer calibration and alignment.
  * Check the heading shown QGC is within within 15 deg truth

* Poor GPS accuracy
  * Check for interference
  * Improve separation and shielding
  * Check flying location for GPS signal obstructions and reflectors \(nearboy tall buildings\)


* Loss of GPS

Determining which of these is the primary casue requires a methodical approach to analysis of the EKF log data:

* Plot the velocty innovation test ratio - [estimator_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vel\_test\_ratio\)

* Plot the horizontal position innovation test ratio - [estimator_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).pos\_test\_ratio\)

* Plot the height innovation test ratio - [estimator_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).hgt\_test\_ratio\)

* Plot the magnetoemrer innovation test ratio - [estimator_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).mag\_test\_ratio\)

* Plot the GPS receier reported speed accuracy - [vehicle_gps_position](https://github.com/PX4/Firmware/blob/master/msg/vehicle_gps_position.msg).s\_variance\_m\_s\)

* Plot the IMU delta angle state estimates - [estimator_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).states\[10\],states\[11\] and states\[12\]

* Plot the EKF internal high frequency vibration metrics:
  * Delta angle coning vibration -[estimator_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vibe\[0\]
  * High frequency delta angle vibration - [estimator_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vibe\[1\]
  * High frequency delta velocity vibration - [estimator_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vibe\[2\]


During normal operation, all the test ratios should remain below 0.5 with only occasional spikes above this as shown in the example below from a successful flight:

![Position, Velocity, Height and Magnetometer Test Ratios](../../assets/ecl/test_ratios_-_successful.png)

The following plot shows the EKF vibration metrics for a multirotor with good isolation. The landing shock and the increased vibration during takeoff and landing can be seen. Insifficient data has been gathered with these metrics to provide specific advice on maximum thresholds.

![](../../assets/ecl/vibration_metrics_-_successful.png)

The above vibration metrics are of limited value as the presence of vibration at a frequency close to the IMU sampling frequency \(1kHz for most boards\) will cause  offsets to appear in the data that do not show up in the high frequency vibration metrics. The only way to detect aliasing errors is in their effect on inertial navigation accuracy and the rise in innovation levels.

In addition to generating large position and velocity test ratios of &gt; 1.0, the different error mechanisms affect the other test ratios in different ways:

### Determination of Excessive Vibration

High vibration levels normally affect vertical positiion and velocity innovations as well as the horizontal components. Magnetometer test levels are only affected to a small extent.

\(insert example plots showing bad vibration here\)

### Determination of Excessive Gyro Bias

Large gyro bias offsets are normally characterised by a change in the value of delta angle bias greater than 5E-4 during flight \(equivalent to ~3 deg/sec\) and can also cause a large increase in the magnetometer test ratio if the yaw axis is affected. Height is normally unaffected other than extreme cases. Switch on bias value of up to 5 deg/sec can be tolerated provided the filter is given time time settle before flying . Pre-flight checks performed by the commander should prevent arming if the position is diverging.

\(insert example plots showing bad gyro bias here\)

### Determination of Poor Yaw Accuracy

Bad yaw alignment causes a velocity test ratio that increases rapidly when the vehicle starts moving due inconsistency in the direction of velocity calculatde by the inertial nav and the  GPS measurement. Magnetometer innovations are slightly affected. Height is normally unaffected.

\(insert example plots showing bad yaw alignment here\)

### Determination of Poor GPS Accuracy

Poor GPS accuracy is normally accompanied by a rise in the reported velocity error of the receiver in conjunction with a rise in innovations. Transient errors due to multipath, obscuration and interference are more common causes. Here is an example of a temporary loss of GPS accuracy where the multi-rotor started drifting away from its loiter location and had to be corrected using the sticks. The rise in [estimator_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vel\_test\_ratio\) to greater than 1 indicates the GPs velocity was inconsistent with other measurements and has been rejected.

![](../../assets/ecl/gps_glitch_-_test_ratios.png)

This is accompanied with rise in the GPS receivers reported velocity accuracy which indicates that it was likely a GPS error.

![](../../assets/ecl/gps_glitch_-_reported_receiver_accuracy.png)

If we also look at the GPS horizontal velocity innovations and innovation variances, we can see the large spike in North velocity innovation that accompanies this GPS 'glitch' event.

![](../../assets/ecl/gps_glitch_-_velocity_innovations.png)

### Determination of GPS Data Loss

Loss of GPS data will be shown by the velocity and position innvoation test ratios 'flat-lining'. If this occurs, check the oher GPS status data in vehicle\_gps\_position for further information.

\(insert example plosts showing loss of GPS data here\)
