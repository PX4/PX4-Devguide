# Switching State Estimators

This page shows you which state estimators are available and how you can switch between them.

> 

## Available Estimators

The available estimators are:

- 
- 
- 
- 

## How to Enable Different Estimators

For multirotors and VTOL use the parameter [SYS_MC_EST_GROUP](../advanced/parameter_reference.md#SYS_MC_EST_GROUP) to choose between the following configurations (LPE is not supported for Fixed Wing).

| SYS_MC_EST_GROUP | Q Estimator |  |  |  |
| ------------------ | ----------- |  |  |  |
| 0                  | enabled     |  |  |  |
| 1                  | enabled     |  |  |  |
| 2                  |             |  |  |  |

> **Note** For FMU-v2 (only) you will also need to build PX4 to specifically include required estimator (e.g. EKF2: `make px4_fmu-v2`, LPE: `make px4_fmu-v2_lpe`). This is required because FMU-v2 is too resource constrained to include both estimators. Other Pixhawk FMU versions include both.