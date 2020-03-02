# 切换状态估计器

此页显示了可用的状态估计器以及如何在它们之间切换。

> **Tip** EKF2 is highly recommended for all purposes (LPE is no longer supported/maintained).

## 可用的估计器

可用的估计器如下：

- **EKF2 attitude, position and wind states estimator** - EKF2 is an extended kalman filter estimating attitude, 3D position / velocity and wind states.
- **LPE position estimator** - The LPE position estimator is an extended kalman filter for 3D position and velocity states.
- **Q attitude estimator** - The attitude Q estimator is a very simple, quaternion based complementary filter for attitude.

## 如何启用不同的估计器

对于多旋翼和 VTOL ，使用参数 [SYS_MC_EST_GROUP](../advanced/parameter_reference.md#SYS_MC_EST_GROUP) 来选择下面的配置（ LPE 不再支持固定翼飞机）。

| SYS_MC_EST_GROUP | Q Estimator | LPE     | EKF2    |
| ------------------ | ----------- | ------- | ------- |
| 1                  | 启用          | enabled |         |
| 2                  |             |         | enabled |
| 3                  | enabled     |         |         |

> **注意** 对于 FMU-v2 （只有它）你需要编译 PX4时指定使用哪个需要的估计器（例如使用 EKF2： `make px4_fmu-v2`，使用 LPE: `make px4_fmu-v2_lpe`）。 这是因为 FMU-v2 不具有足够的资源同时包含这两个估计器。 其他的 Pixhawk FMU 版本同时拥有2个估计器。