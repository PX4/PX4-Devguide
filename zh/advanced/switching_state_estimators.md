---
translated_page: https://github.com/PX4/Devguide/blob/master/en/advanced/switching_state_estimators.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# 切换状态估计器


本文主要介绍PX4中有哪些可用的状态估计器以及用户该如何在不同的估计器间进行切换。

## 可用的估计器


**1. Q attitude estimator(四元数姿态估计)**

四元数姿态估计方法非常简单，就是基于四元数的姿态互补滤波器。

**2. INAV position estimator(惯导位置估计)**

惯导位置估计使用互补滤波器对三维位置以及速度进行估计。。

**3. LPE position estimator(LPE位置估计)**

LPE (Local Position Estimator) 位置估计使用扩展卡尔曼滤波器对三维位置以及速度进行估计。

**4. EKF2 attitude, position and wind states estimator (EKF2姿态，位置以及风速估计)**

EKF2使用扩展卡尔曼滤波器进行三维的姿态，位置/速度以及风的状态进行估计。

**5. EKF attitude, position and wind states estimator (depricated)**(EKF姿态，位置以及风速估计(已过时))
（即固件参数列表中的[Attitude EKF estimator](https://pixhawk.org/firmware/parameters#attitude_ekf_estimator)和[Position Estimator](https://pixhawk.org/firmware/parameters#position_estimator))

这是一个类似于EKF2的扩展卡尔曼滤波器。然而，很快它就将完全由EKF2代替。

此滤波器仅用于固定翼。

## **如何使能不同的估计器**

对于多旋翼和垂直起降飞行器，使用参数**SYS_MC_EST_GROUP**在下列配置中进行选择。


> 目前只有已过时的EKF估计器被用于非垂直起降的飞行器。它将很快被EKF2代替。



| SYS_MC_EST_GROUP | Q Estimator | INAV | LPE  | EKF2 |
| ---------------- | ----------- | ---- | ---- | ---- |
| 0                | 使能          | 使能   |      |      |
| 1                | 使能          |      | 使能   |      |
| 2                |             |      |      | 使能   |

