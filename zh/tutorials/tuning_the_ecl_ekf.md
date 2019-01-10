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
* \[4 ... 6\] 速度 NED\（m/s\）
* \[7 ... 9\] 位置 NED\（m\）
* \[10 ... 12\] IMU delta 角度偏差 XYZ\（rad\）
* \[13 ... 15\] IMU delta 速度偏差 XYZ\（m/s\）
* \[16 ... 18\] 地球磁场 NED\（gauss\）
* \[19 ... 21\] 体磁场 XYZ \（gauss\）
* \[22 ... 23\] 风速 NE\（m/s\）
* \[24 ... 32\] 未使用

### 状态变量

请参阅 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) 中的协方差\[28\]。 协方差\[28\] 的索引图如下：

* \[0 ... 3\] 四元数
* \[4 ... 6\] 速度 NED\（m/s\）^2
* \[7 ... 9\] 位置 NED\（m^2\）
* \[10 ... 12\] IMU delta 角度偏差 XYZ\（rad^2\）
* \[13 ... 15\] IMU delta 速度偏差 XYZ\（m/s\）^2
* \[16 ... 18\] 地球磁场 NED\（gauss^2\）
* \[19 ... 21\] 体磁场 XYZ\（gauss^2\）
* \[22 ... 23\] 风速 NE\（m/s\）^2
* \[24 ... 28\] 未使用

### 新息（ t 时刻的实际量测值 与 t-1 时刻对 t 时刻量测的估计值与之间的差）

* 磁力计 XYZ\（gauss\）：请参阅 [ekf2\__innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg) 中的 mag\_innov\[3\]。
* 偏航角\（rad\）：请参阅 [ekf2\__innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg) 中的标题 \_innov。
* 速度和位置创新：请参阅 [ekf2\__innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg) 中的 vel\_pos\_innov\[6\]。 Vel\_pos\_innov\[6\] 的索引图如下： 
  * \[0 ... 2\] 速度 NED\（m/s\）
  * \[3 ... 5\] 位置 NED\（m\）
* 真实空速\（m/s\）：请参阅 [ekf2\__innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg) 中的空速 \_innov。
* 合成侧滑\（rad\）：请参阅 [ekf2\__innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg) 中的 beta \_innov。
* 光流 XY\（rad/sec\）：请参阅 [ekf2\__innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg) 中的 flow\_innov。
* 高于地面的高度\（m\）：请参阅 [ekf2\__innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg) 中的 hagl \_innov。

### 新息协方差

* 磁力计 XYZ\（gauss^2\）：请参阅 [ekf2\__innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg) 中的 mag\_innov\_var\[3\]。
* 偏航角\（rad^2\）：请参阅 ekf2\_innovations 消息中的标题 \_innov\_var。
* 速度和位置创新：请参阅 [ekf2\__innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg) 中的 vel\_pos\_innov\_var\[6\]。 Vel\_pos\_innov\_var\[6\] 的索引映射如下： 
  * \[0 ... 2\] 速度 NED\（m/s\）^2
  * \[3 ... 5\] 位置 NED\（m^2\）
* 真空速\（m/s\）^2：请参阅 [ekf2\__innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg) 中的空速 \_innov\_var。
* 合成侧滑\（rad\）：请参阅 [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg) 中的beta\_innov\_var。
* 光流 XY\（rad/sec\）^2：请参阅 [ekf2\__innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg) 中的 flow\_innov\_var。
* 高于地面的高度\（m^2\）：请参阅 [ekf2\__innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg) 中的 hagl\_innov\_var。

### 输出互补滤波器

输出互补滤波器用于将状态从融合时间范围向前传播到当前时间。 要检查在融合时间范围内测量的角度，速度和位置跟踪误差的大小，请参阅 ekf2\_innovations 消息中的 output\_tracking\_error\[3\]。 索引图如下：

* \[0\] 角度跟踪误差幅度\（rad\）
* \[1\] 速度跟踪误差幅度\（m/s\）。 可以使用 [EKF2_TAU_VEL](../advanced/parameter_reference.md#EKF2_TAU_VEL) 参数调整速度跟踪时间常数。 减小此参数可减少稳态误差，但会增加 NED 速度输出上的观察噪声量。
* \[2\] 位置跟踪误差幅度\（m\）。 可以使用 [EKF2_TAU_POS](../advanced/parameter_reference.md#EKF2_TAU_POS) 参数调整位置跟踪时间常数。 减小此参数可减少稳态误差，但会增加 NED 位置输出上的观察噪声量。

### EKF 错误

EKF 包含针对严重条件状态和协方差更新的内部错误检查。 请参阅 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) 中的 filter\_fault\_flags。

### 观察错误

有两类观察错误：

* 数据丢失。 一个例子是测距仪无法提供返回。
* 创新，即状态预测和传感器观察之间的差异是过度的。 这种情况的一个例子是过度振动导致大的垂直位置误差，导致气压计高度测量被拒绝。

这两者都可能导致观察数据被拒绝足够长的时间以使 EKF 使用传感器观察来尝试重置状态。 所有观察结果均对创新进行了统计置信度检查。 检查的标准偏差数由每种观察类型的 EKF2\_ \*\_GATE 参数控制。

测试级别在 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) 中可用，如下所示：

* mag\_test\__tio：最大磁力计创新组件与创新测试限制的比率
* vel\_test\__tio：最大速度创新组件与创新测试限制的比率
* pos\_test\__tio：最大水平位置创新组件与创新测试限制的比率
* hgt\test\ratio：垂直位置创新与创新测试限制的比率
* tas\_test\__tio：真正的空速创新与创新测试极限的比率
* hagl\_test\__tio：地面创新高度与创新测试限制的比率

有关每个传感器的二进制通过/失败摘要，请参阅 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) 中的 innovation\_check\__flags。

### GPS 质量检查

在开始 GPS 辅助之前，EKF 应用了许多 GPS 质量检查。 这些检查由 [EKF2_GPS_CHECK](../advanced/parameter_reference.md#EKF2_GPS_CHECK) 和 `EKF2_REQ _ * ` 参数控制。 这些检查的通过/失败状态记录在 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).gps\_check\_fail\__flags 消息中。 所有必需的 GPS 检查通过后，此整数将为零。 如果 EKF 未开始 GPS 对齐，请根据 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) 中的位掩码定义 gps\_check\_fail\_flags 检查整数的值。

### EKF 数值误差

EKF 对其所有计算使用单精度浮点运算，并使用一阶近似来推导协方差预测和更新方程，以降低处理要求。 这意味着当重新调整 EKF 时可能遇到协方差矩阵运算变得严重条件足以引起状态估计中的分歧或显着错误的条件。

为防止这种情况，每个协方差和状态更新步骤都包含以下错误检测和更正步骤：

* 如果创新方差小于观察方差\（这需要一个不可能的负状态方差）或协方差更新将为任何一个状态产生负方差，那么： 
  * 跳过状态和协方差更新
  * 协方差矩阵中的相应行和列被重置
  * 失败记录在 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) filter\_fault\__flags 消息中
* 状态方差\（协方差矩阵中的对角线\）被约束为非负的。
* 上限应用于状态方差。
* 协方差强制在协方差矩阵上。

重新调整过滤器后，特别是需要减少噪声变量的重新调整，应检查 estimator\_status.gps\_check\_fail\__flags的值，以确保它保持为零。

## 如果高度估计发散了怎么办?

在飞行期间 EKF 高度偏离 GPS 和高度计测量的最常见原因是由振动引起的 IMU 测量的削波和/或混叠。 如果发生这种情况，则数据中应显示以下迹象

* [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).vel\_pos\_innov\[2\] 和 [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).vel\_pos\_innov\[5\] 将具有相同的符号。
* [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).hgt\_test\__tio 将大于 1.0

建议第一步是确保使用有效的隔离安装系统将无人机与机身隔离。 隔离安装座具有 6 个自由度，因此具有 6 个谐振频率。 作为一般规则，隔离支架上的自动驾驶仪的 6 个共振频率应高于 25Hz，以避免与无人机动力学相互作用并低于电动机的频率。

如果谐振频率与电动机或螺旋桨叶片通过频率一致，则隔离安装件会使振动更严重。

通过进行以下参数更改，可以使 EKF 更加抵抗振动引起的高度发散：

* 将主要的高度传感器的创新通道的值加倍。 如果使用气压高度，则 [EKF2_BARO_GATE](../advanced/parameter_reference.md#EKF2_BARO_GATE)。
* 最初将 [EKF2_ACC_NOISE](../advanced/parameter_reference.md#EKF2_ACC_NOISE) 的值增加到 0.5。 如果仍然出现分歧，则进一步增加 0.1，但不要超过 1.0。

Note 这些变化的影响将使 EKF 对 GPS 垂直速度和气压的误差更敏感。

## 如果位置估计发散了应该怎么办?

位置散度的最常见原因是：

* 高振动水平。 
  * 通过改进无人机的机械隔离来解决。
  * 增加 [EKF2_ACC_NOISE](../advanced/parameter_reference.md#EKF2_ACC_NOISE) 和 [EKF2_GYR_NOISE](../advanced/parameter_reference.md#EKF2_GYR_NOISE) 的值会有所帮助，但确实会使 EKF 更容易受到 GPS 故障的影响。
* 大陀螺仪偏置偏移。 
  * 通过重新校准陀螺仪来修复。 检查过高的温度灵敏度\（在冷启动预热期间 &gt; 3 度/秒的偏压变化，如果受到隔热影响，则更换传感器以减缓温度变化的速度。
* 偏航偏差 
  * 检查磁力计校准和校准。
  * 检查显示的标题 QGC 是否在 15 度以内
* GPS 精度差 
  * 检查是否有干扰
  * 改善分离和屏蔽
  * 检查飞行位置是否有 GPS 信号障碍物和反射器\（附近的高层建筑\）
* GPS 丢失

确定哪些是主要原因需要有条理的方法来分析 EKF 日志数据：

* 绘制速度创新测试比率-[estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vel\_test\_ratio
* 绘制水平位置创新测试比率-[estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).pos\_test\_ratio
* 绘制高度创新测试比率-[estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).hgt\_test\_ratio
* 绘制磁力计创新测试比率-[estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).mag\_test\_ratio
* 绘制 GPS 接收器报告的速度精度-[vehicle\_gps\_position](https://github.com/PX4/Firmware/blob/master/msg/vehicle_gps_position.msg).s\_variance\_m\_s
* 绘制 IMU delta 角度状态估计值-[estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).states\[10\]，状态\[11\]和状态\[12\]
* 绘制 EKF 内部高频振动指标： 
  * 三角形锥度振动-[estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vibe\[0\]
  * 高频 delta 角振动-[estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vibe\[1\]
  * 高频 delta 速度振动-[estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vibe\[2\]

在正常操作期间，所有测试比率应保持在 0.5 以下，并且只有偶然的峰值高于此值，如下面成功飞行中的示例所示：

![Position, Velocity, Height and Magnetometer Test Ratios](../../assets/ecl/test_ratios_-_successful.png)

下图显示了具有良好隔离的多旋翼飞行器的 EKF 振动指标。 可以看到着陆冲击和起飞和着陆期间增加的振动。 使用这些指标收集的数据不足，无法提供有关最大阈值的具体建议。

![](../../assets/ecl/vibration_metrics_-_successful.png)

上述振动指标的价值有限，因为在接近 IMU 采样频率的频率下存在振动\（大多数电路板为 1kHz）将导致在高频振动指标中未显示的数据中出现偏移。 检测混叠误差的唯一方法是它们对惯性导航精度和创新水平的提高。

除了生成 &gt; 1.0 的大位置和速度测试比率外，不同的误差机制还以不同的方式影响其他测试比率：

### 振动过大的测定

高振动水平通常会影响垂直位置和速度创新以及水平组件。 磁强计测试水平仅受到很小程度的影响。

\（在此处插入显示不良振动的示例图\）

### 过度陀螺偏压的测定

大陀螺偏置偏移通常的特征是在飞行期间 delta 角度偏差值的变化大于 5E-4（相当于~3 度/秒），并且如果偏航轴受到影响，也会导致磁强计测试比大幅增加。 除极端情况外，高度通常不受影响。 如果滤波器在飞行前给定时间稳定，则可以容忍接通最高 5 度/秒的偏置值。 如果位置发散，飞手进行的飞行前检查应防止解锁。

\（插入示例曲线显示这里的陀螺偏差\）

### 确定较差的偏航精度

由于惯性导航和 GPS 测量计算出的速度方向不一致，因此不良偏航对准导致无人机开始移动时速度测试比率迅速增加。 Magnetometer innovations are slightly affected. 高度通常不受影响。

\（插入示例图显示错误的偏航对齐此处\）

### GPS 精度差的确定

Poor GPS accuracy is normally accompanied by a rise in the reported velocity error of the receiver in conjunction with a rise in innovations. 由多径，遮蔽和干扰引起的瞬态误差是更常见的原因。 下面是一个暂时失去 GPS 精度的例子，其中多旋翼开始从其游荡位置漂移并且必须使用摇杆进行校正。 [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vel\_test\_ratio 的上升值大于 1 表示 GPS 速度与其他测量值不一致，并且已被拒绝。

![](../../assets/ecl/gps_glitch_-_test_ratios.png)

这伴随着 GPS 接收器报告的速度精度的上升，这表明它可能是 GPS 误差。

![](../../assets/ecl/gps_glitch_-_reported_receiver_accuracy.png)

If we also look at the GPS horizontal velocity innovations and innovation variances, we can see the large spike in North velocity innovation that accompanies this GPS 'glitch' event.

![](../../assets/ecl/gps_glitch_-_velocity_innovations.png)

### GPS 数据丢失的确定

Loss of GPS data will be shown by the velocity and position innovation test ratios 'flat-lining'. 如果发生这种情况，请检查 `vehicle_gps_position` 中的其他 GPS 状态数据以获取更多信息。

下图显示了 NED GPS 速度创新 `ekf2_innovations_0.vel_pos_innov[0 ... 2]`，GPS NE 位置创新 `ekf2_innovations_0.vel_pos_innov[3 ... 4] `和使用 SITL Gazebo 从模拟 VTOL 生成的 Baro 垂直位置创新 `ekf2_innovations_0.vel_pos_innov [5] `。

模拟的 GPS 在 73 秒时失锁。 Note the NED velocity innovations and NE position innovations 'flat-line' after GPs is lost. Note that after 10 seconds without GPS data, the EKF reverts back to a static position mode using the last known position and the NE position innovations start to change again.

![GPS Data Loss - in SITL](../../assets/ecl/gps_data_loss_-_velocity_innovations.png)