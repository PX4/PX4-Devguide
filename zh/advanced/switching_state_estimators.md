# 切换状态估计器

此页显示了可用的状态估计器以及如何在它们之间切换。

> **提示**强烈建议将 ekf2 用于所有用途 （不再维护 lpe）。

## 可用的估计器

可用的估计器如下：

- **Q attitude estimator** - The attitude Q estimator is a very simple, quaternion based complementary filter for attitude.
- **INAV position estimator** - The INAV position estimator is a complementary filter for 3D position and velocity states.
- **LPE position estimator** - The LPE position estimator is an extended kalman filter for 3D position and velocity states.
- **EKF2 attitude, position and wind states estimator** - EKF2 is an extended kalman filter estimating attitude, 3D position / velocity and wind states.

## How to Enable Different Estimators

For multirotors and VTOL use the parameter [SYS_MC_EST_GROUP](../advanced/parameter_reference.md#SYS_MC_EST_GROUP) to choose between the following configurations (LPE is not supported for Fixed Wing).

| SYS_MC_EST_GROUP | Q Estimator | INAV    | LPE     | EKF2    |
| ------------------ | ----------- | ------- | ------- | ------- |
| 0                  | enabled     | enabled |         |         |
| 1                  | enabled     |         | enabled |         |
| 2                  |             |         |         | enabled |

> **Note** For FMU-v2 (only) you will also need to build PX4 to specifically include required estimator (e.g. EKF2: `make px4_fmu-v2`, LPE: `make px4_fmu-v2_lpe`). This is required because FMU-v2 is too resource constrained to include both estimators. Other Pixhawk FMU versions include both.