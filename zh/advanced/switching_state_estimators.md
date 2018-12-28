# 切换状态估计器

此页显示了可用的状态估计器以及如何在它们之间切换。

> **提示**强烈建议将 ekf2 用于所有用途 （不再维护 lpe）。

## 可用的估计器

可用的估计器如下：

- ** Q attitude estimator ** - attitude Q estimator 是一种用于姿态的、简单的、基于四元数的互补滤波器。
- **INAV position estimator** - INAV position estimator 是一种用于三维位置与速度状态的互补滤波器。
- **LPE position estimator** - LPE position estimator 是一种用于三维位置与速度状态的扩展卡尔曼估计器。
- **EKF2 attitude, position and wind states estimator** - EKF2 是一种用于估计姿态、三维速度/速度与风的状态的扩展卡尔曼滤波器。

## 如何使能不同的估计器

对于多旋翼和 VTOL ，使用参数 [SYS_MC_EST_GROUP](../advanced/parameter_reference.md#SYS_MC_EST_GROUP) 来选择下面的配置（ LPE 不再支持固定翼飞机）。

| SYS_MC_EST_GROUP | Q Estimator | INAV    | LPE     | EKF2    |
| ------------------ | ----------- | ------- | ------- | ------- |
| 0                  | enabled     | enabled |         |         |
| 1                  | enabled     |         | enabled |         |
| 2                  |             |         |         | enabled |

> **Note** For FMU-v2 (only) you will also need to build PX4 to specifically include required estimator (e.g. EKF2: `make px4_fmu-v2`, LPE: `make px4_fmu-v2_lpe`). This is required because FMU-v2 is too resource constrained to include both estimators. Other Pixhawk FMU versions include both.