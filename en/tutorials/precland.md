# Precision Landing

PX4 supports precision landing for multicopters with the [IRLock](https://irlock.com/products/ir-lock-sensor-precision-landing-kit) sensor and a downward facing range sensor.

A precision landing can be initiated by entering the precision land flight mode, or as part of a mission uploaded via Mavlink.

Besides using the beacon (landing target) to control the vehicle to land on top of it, it can be used to improve the vehicle's localization, if the beacon is stationary.
This is controlled by a parameter (see below).
Note: This is currently only possible with LPE.

The `landing_target_estimator` module fuses the IRLock and range measurements to estimate the beacon's relative position and orientation w.r.t. the vehicle.
This information is then used by LPE to improve the state estimate (if enabled), as well as the precision land controller.

## Setup
### Hardware setup
Install the IRLock sensor according to the official [IRLock guide](https://irlock.readme.io/v2.0/docs). Ensure that the sensor's x axis is aligned with the vehicle's y axis and the sensor's y axis aligned with the vehicle's -x direction. (This is the case if the camera is pitched down 90 deg from facing forward.)

Install a [range sensor](https://docs.px4.io/en/getting_started/sensor_selection.html#distance).
Note that many infrared based range sensors do not perform well in the presence of the IRLock beacon.
A sensor that has been found to work well is the LidarLite v3.
Also refer to the IRLock guide for suggested sensors.

Power and place the IRLock beacon where you want the vehicle to land.

### Software setup
Precision landing requires the modules `irlock` and `landing_target_estimator` which are not included in the firmware by default.
Add them by adding or uncommenting the following lines in the relevant [config](https://github.com/PX4/Firmware/tree/master/cmake/configs) of your flight controller:

```
drivers/irlock
modules/landing_target_estimator
```

Start the two modules by [customizing the system startup](../advanced/system_startup.html#starting-additional-applications).

## Parameters
Precision landing is configured with parameters of the `landing_target_estimator` and `navigator`, which are found in the "Landng target estimator" and "Precision land" groups, respectively.
The most important parameters are discussed below.

### Estimation parameters
The parameter `LTEST_MODE` determines if the beacon is assumed to be stationary or moving.
If `LTEST_MODE` is set to moving (e.g. it is installed on a vehicle on which the multicopter is to land), beacon measurements are only used to generate position setpoints in the precision landing controller.
If `LTEST_MODE` is set to stationary, the beacon measurements are also used by LPE.

The parameters `LTEST_SCALE_X` and `LTEST_SCALE_Y` can be used to scale beacon measurements before they are used to estimate the beacon's position and velocity relative to the vehicle.
Measurement scaling may be necessary due to lens distortions of the IRLock sensor.
Note that `LTEST_SCALE_X` and `LTEST_SCALE_Y` are considered in the sensor frame, not the vehicle frame.

To calibrate these scale parameters, set `LTEST_MODE` to moving, fly your multicopter above the beacon and perform forward-backward and left-right motions with the vehicle, while [logging](../log/logging.html#configuration) `landing_target_pose` and `vehicle_local_position`.
Then, compare `landing_target_pose.vx_rel` and `landing_target_pose.vy_rel` to `vehicle_local_position.vx` and `vehicle_local_position.vy`, respectively (both measurements are in NED frame). If the estimated beacon velocities are consistently smaller or larger than the vehicle velocities, adjust the scale parameters to compensate.

If you are observing slo sideways oscillations of the vehicle while doing a precision landing with `LTEST_MODE` set to stationary, the beacon measurements are likely scaled too high and you should reduce the scale parameter in the relevant direction.

## Performing a precision landing
### Via command
Precision landing can be initiated through the command line interface with
```
commander mode auto:precland
```

### In a mission
Precision landing can be initiated as part of a [mission](https://dev.px4.io/en/qgc/#planning-missions) by appropriately setting `param2` of a `MAV_CMD_NAV_LAND`.
Depeding on the value of `param2`, the landing behavior is different.

- `param2` = 0: Normal landing without using the beacon.
- `param2` = 1: Opportunistic precision landing. If the beacon is visible when the land waypoint is reached, perform a precision landing, otherwise fall back to a normal landing.
- `param2` = 2: Required precision landing. If the beacon is not visible when the land waypoint is reached, climb to a specified search altitude (`PLD_SRCH_ALT`). As soon as the beacon is found, perform a precision landing. If the beacon is not seen even at the search altitude, fall back to a normal landing after a search timeout (`PLD_SRCH_TOUT`).
