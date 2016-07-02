# Optical flow and LIDAR-Lite

This page shows you how to set up the PX4Flow and a LIDAR-Lite for position estimation.


## Selecting an Estimator

Two estimators support optical flow based navigation, LPE and INAV. There are benefits to both, but LPE is currently recommended for new users as it has the most testing and is the most robust.

### Local Position Estimator (LPE)

LPE is an Extended Kalman Filter based estimator for position and velocity states. It uses inertial navigation and is similar to the INAV estimator below but it dynamics calculates the Kalman gain based on the state covariance. It also is capable of detecting faults, which is beneficial for sensors like sonar which can return invalid reads over soft surfaces.

Below is a plot of an autonomous mission using optical flow. GPS is not used to estimate the vehicle position but is plotted for a ground truth comparison.

![](images/lpe/lpe_flow_vs_gps.png)

#### LPE Flight Videos

Due to the ability of LPE to more robustly handle optical flow based estimation, it is capable of performing auto missions at low altitude (less than 3 meters) and slow speed (less than 2 m/s). The main criterion is that the camera image is in focus and the vehicle doesn't lean significantly during the flight.

* [indoor](https://www.youtube.com/watch?v=CccoyyX-xtE) 
* [outdoor](https://www.youtube.com/watch?v=Ttfq0-2K434)

### Inertial Navigation Extimator (INAV)

INAV has a fixed gain matrix for correction and can be viewed as a steady state Kalman filter. It has the lowest computational cost of all position estimators.

#### INAV Flight Videos
* [indoor](https://www.youtube.com/watch?v=MtmWYCEEmS8) 
* [outdoor](https://www.youtube.com/watch?v=4MEEeTQiWrQ)

## Hardware
![](images/hardware/px4flow.png)
![](images/hardware/lidarlite.png)


The PX4Flow has to point towards the ground and can be connected using the I2C port on the pixhawk.
For the connection of the LIDAR-Lite please refer to [this](https://pixhawk.org/peripherals/rangefinder?s[]=lidar) page.
For best performance make sure the PX4Flow is attached at a good position and is not exposed to vibration. (preferably on the down side of the quad-rotor).

![](images/hardware/flow_lidar_attached.jpg)

## Sensor Parameters
All the parameters can be changed in QGroundControl
* SENS_EN_LL40LS
	Set to 1 to enable lidar-lite distance measurements

## Estimator Parameters

![](images/hardware/px4flow_offset.png)

### Local Position Estimator (LPE)

The local position estimator will automatically fuse lidar and optical flow data when present.

* LPE_FLOW_OFF_Z - This is the offset of the optical flow camera from the center of mass of the vehicle. This measures positive down and defaults to zero. This can be left zero for most typical configurations where the z offset is negligible.
* LPE_FLW_XY - Flow standard deviation in meters.
* LPW_FLW_QMIN - Minimum flow quality to accept measurement.
* LPE_SNR_Z -Sonar standard deviation in meters.
* LPE_SNR_OFF_Z - Offset of sonar sensor from center of mass.
* LPE_LDR_Z - Lidar standard deviation in meters.
* LPE_LDR_Z_OFF -Offset of lidar from center of mass.


### INAV
* INAV_LIDAR_EST
	Set to 1 to enable altitude estimation based on distance measurements
* INAV_FLOW_DIST_X and INAV_FLOW_DIST_Y
	These two values (in meters) are used for yaw compensation.
	The offset has to be measured according to the following figure:
	In the above example the offset of the PX4Flow (red dot) would have a negative X offset and a negative Y offset.
* INAV_LIDAR_OFF
	Set a calibration offset for the lidar-lite in meters. The value will be added to the measured distance.


## Advanced

For advanced usage/development the following parameters can be changed as well. Do NOT change them if you do not know what you are doing!

* INAV_FLOW_W
	Sets the weight for the flow estimation/update
* INAV_LIDAR_ERR
	Sets the threshold for altitude estimation/update in meters. If the correction term is bigger than this value, it will not be used for the update.
