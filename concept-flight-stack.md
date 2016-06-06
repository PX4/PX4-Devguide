# PX4 Flight Stack

The PX4 flight stack is a collection of guidance, navigation and control algorithms for autonomous drones. It includes controllers for fixed wing, multirotor and VTOL airframes as well as estimators for attitude and position.

## Estimation and Control Architecture

The diagram below shows an example implementation of the typical blocks. Depending on the vehicle some of these can be also combined into a single application (e.g. when a model predictive controller for a specific vehicle is wanted).

```mermaid
graph TD;
  pos_ctrl-->att_ctrl;
  att_ctrl-->mixer;
  inertial_sensors-->attitude_estimator;
  inertial_sensors-->position_estimator;
  GPS-->position_estimator;
  computer_vision-->position_estimator;
  position_estimator-->navigator;
  position_estimator-->attitude_estimator;
  position_estimator-->pos_ctrl;
  attitude_estimator-->att_ctrl;
```
