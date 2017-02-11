# Switching State Estimators
This page shows you which state estimators are available and how you can switch between them.

## Available estimators

**1. Q attitude estimator**

The attitude Q estimator is a very simple, quaternion based complementary filter for attitude.

**2. INAV position estimator**

The INAV position estimator is a complementary filter for 3D position and velocity states.


**3. LPE position estimator**

The LPE position estimator is an extended kalman filter for 3D position and velocity states.

**4. EKF2 attitude, position and wind states estimator**

EKF2 is an extended kalman filter estimating attitude, 3D position / velocity and wind states.

**4. EKF attitude, position and wind states estimator (deprecated)**
This is an extended kalman filter similar to EKF2. However, it will soon be replaced completely by EKF2.
This filter was only used for fixed wings.

## How to enable different estimators
For multirotors and VTOL use the parameter **SYS_MC_EST_GROUP** to chose between the following configurations.


| SYS_MC_EST_GROUP | Q Estimator| INAV | LPE | EKF2 |
| --- | --- | --- | --- | --- |
| 0 | enabled | enabled | | |
| 1 | enabled |  | enabled | |
| 2 |  |  | | enabled |


