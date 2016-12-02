# Using the ecl EKF
This tutorial answers common questions about use of the ECL EKF algorithm. 

## What is the ecl EKF?
The ECL (Estimation and Control Library) uses an Extended Kalman Filter algorithm to processe sensor measurements and provide an estimate of the following states:

* Quaternion defining the rotation from earth to body frame
* Velocity at the IMU North,East,Down (m/s)
* Position at the IMU North,East,Down (m)
* IMU delta angle bias estimates X,Y,Z (rad)
* IMU delta velocity bias estimates X,Y,Z(m/s)
* Earth Magnetic field components North, East,Down (gauss)
* Vehicle body frame magnetic field bias X,Y,Z (gauss)
* Wind velocity North,East (m/s)

The EKF runs on a delayed 'fusion time horizon' to allow for different time delays on each measurement relative to the IMU. Data for each sensor is FIFO buffered and retrieved from the buffer by the EKF to be used at the correct time. The delay compensation for each sensor is controlled by the EKF2_*_DELAY parameters.

A complementary filter is used to propagate the states forward from the 'fusion time horizon' to current time using the buffered IMU data. The time constant for this filter is controlled by the EKF2_TAU_VEL and EKF2_TAU_POS parameters.

Note: The 'fusion time horizon' delay and length of the buffers is determined by the largest of the EKF2_*_DELAY parameters. If a sensor is not being used, it is recommended to set its time delay to zero. Reducing the 'fusion time horizon' delay reduces errors in the complementary filter used to propagate states forward to current time.

The position and velocity states are adjusted to account for the offset between the IMU and the body frame before they are output to the control loops. The position of the IMU relative to the body frame is set by the EKF2_IMU_POS_X,Y,Z parameters.

## What sensor measurements does it use?
The EKF has different modes of operation that use different combinations of sensor measurements:

The first is a mode entered into on start-up that provides rotation, vertical velocity,  vertical position, IMU delta angle bias and IMU delta velocity bias estimates. It uses the following measurements which are mandatory for all modes of operation:

* Three axis body fixed Inertial Measurement unit delta angle and delta velocity data at a minimum rate of 100Hz. Note: Coning corrections should be applied to the IMU delta angle data before it is used by the EKF.
* Three axis body fixed magnetometer data OR external vision system pose data at a minimum rate of 5Hz 
* A source of height data - either GPS, barometric pressure, range finder or external vision at a minimum rate of 5Hz. Note: The primary source of height data is controlled by the EKF2_HGT_MODE parameter. 

If these measurements are not present, the EKF will not start. When these measurements have been detected, the EKF will initialise the states and complete the tilt and yaw alignment. When tilt and yaw alignment is complete, the EKF can then transition to other modes of operation  enabling use of additional sensor data:

* GPS North, East, Down position and velocity. GPS measurements will be used for position and velocity if the following conditions are met:
 * GPS use is enabled via setting of the EKF2_AID_MASK parameter.
 * GPS quality checks have passed. These checks are controlled by the EKF2_GPS_CHECK and EKF2_REQ<> parameters. 
 * GPS height can be used directly by the EKF via setting of the EKF2_HGT_MODE parameter.
* Range finder distance to ground. Range finder data is used a by a single state filter to estimate the vertical position of the terrain relative to the height datum. 
 * If operating over a flat surface that can be used as a zero height datum, the range finder data can be used directly by the EKF to estimate height by setting the EKF2_HGT_MODE parameter to 2. 
* Equivalent Airspeed (EAS). This data can be used to estimate wind velocity and reduce drift when GPS is lost by setting EKF2_ARSP_THR to a positive value representing the minimum speed for airspeed measurements to be considered valid.
* Optical Flow. Data from an attached optical flow sensor will be used if the following conditions are met:
 * Valid range finder data is available.
 * Bit position 1 in the EKF2_AID_MASK parameter is true.
 * The quality measure returned by the flow sensor is greater than the minimum requirement set by the EKF2_OF_QMIN parameter
* External vision system horizontal position data will be used if bit position 3 in the EKF2_AID_MASK parameter is true.
* External vision system vertical position data will be used if the EKF2_HGT_MODE parameter is set to 3.
* External vision system pose data will be used for yaw estimation if bit position 4 in the EKF2_AID_MASK parameter is true.

## How do I use the 'ecl' library EKF?
Set the SYS_MC_EST_GROUP parameter to 2 to use the ecl EKF.

## How do I check the EKF perfomrance?
EKF outputs, states and status data are published to a number of uORB topics which are logged to SD card during flight.

Output Data:

* Attitude output data: Refer to vehicle_attitude.msg for definitions.
* Local position output data: Refer to vehicle_local_position.msg for definitions.
* Control loop feedback data: Refer to control_state.msg for definitions.
* Global (WGS-84) output data: Refer to vehicle_global_position.msg for definitions.
* Wind velocity output data: Refer to wind_estimate.msg for definitions.

Status Data:

* 

