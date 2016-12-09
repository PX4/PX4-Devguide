# Preflight Sensor and EKF Checks
The commander module performs a number of preflight sensor quality and EKF checks which are controlled by the COM_ARM<> parameters. If these checks fail, the motors are prevented from arming and the following error messages are produced:

* PREFLIGHT FAIL: EKF HGT ERROR
 * This error is produced when the IMU and height measurement data are inconsistent.
 * Perform an accel and gyro calibration and restart the vehicle. If the error persists, check the height sensor data for problems.
 * The check is controlled by the COM_ARM_EKF_HGT parameter.
* PREFLIGHT FAIL: EKF VEL ERROR
 * This error is produced when the IMU and GPS velocity measurement data are inconsistent. 
 * Check the GPS velocity data for un-realistic data jumps. If GPS quality looks OK, perform an accel and gyro calibration and restart the vehicle.
 * The check is controlled by the COM_ARM_EKF_VEL parameter.
* PREFLIGHT FAIL: EKF HORIZ POS ERROR
 * This error is produced when the IMU and position measurement data (either GPS or external vision) are inconsistent. 
 * Check the position sensor data for un-realistic data jumps. If data quality looks OK, perform an accel and gyro calibration and restart the vehicle.
 * The check is controlled by the COM_ARM_EKF_POS parameter.
* PREFLIGHT FAIL: EKF YAW ERROR
 * This error is produced when the yaw angle estimated using gyro data and the yaw angle from the magnetometer or external vision system are inconsistent.
 * Check the IMU data for large yaw rate offsets and check the magnetometer alignment and calibration.
 * The check is controlled by the COM_ARM_EKF_POS parameter
* PREFLIGHT FAIL: EKF HIGH IMU ACCEL BIAS
 * This error is produced when the IMU accelerometer bias estimated by the EKF is excessive. 
 * The check is controlled by the COM_ARM_EKF_AB parameter.
* PREFLIGHT FAIL: EKF HIGH IMU GYRO BIAS
 * This error is produced when the IMU gyro bias estimated by the EKF is excessive. 
 * The check is controlled by the COM_ARM_EKF_GB parameter.
* PREFLIGHT FAIL: ACCEL SENSORS INCONSISTENT - CHECK CALIBRATION
 * This error message is produced when the acceleration measurements from different IMU units are inconsistent.
 * This check only applies to boards with more than one IMU.
 * The check is controlled by the COM_ARM_IMU_ACC parameter.
* PREFLIGHT FAIL: GYRO SENSORS INCONSISTENT - CHECK CALIBRATION
 * This error message is produced when the angular rate measurements from different IMU units are inconsistent.
 * This check only applies to boards with more than one IMU.
 * The check is controlled by the COM_ARM_IMU_GYR parameter.

##COM_ARM_WO_GPS
The COM_ARM_WO_GPS parameter controls whether arming is allowed without a GPS signal. This parameter must be set to 0 to allow arming when there is no GPS signal present. Arming without GPS is only allowed if the flight mode selected does not require GPS.
##COM_ARM_EKF_POS
The COM_ARM_EKF_POS parameter controls the maximum allowed inconsistency between the EKF inertial measurements and position reference (GPS or external vision). The default value of 0.5 allows the differences to be no more than 50% of the maximum tolerated by the EKF and provides some margin for error increase when flight commences.
##COM_ARM_EKF_VEL
The COM_ARM_EKF_VEL parameter controls the maximum allowed inconsistency between the EKF inertial measurements and GPS velocity measurements. The default value of 0.5 allows the differences to be no more than 50% of the maximum tolerated by the EKF and provides some margin for error increase when flight commences.
##COM_ARM_EKF_HGT
The COM_ARM_EKF_HGT parameter controls the maximum allowed inconsistency between the EKF inertial measurements and height measurement (Baro, GPS, range finder or external vision). The default value of 0.5 allows the differences to be no more than 50% of the maximum tolerated by the EKF and provides some margin for error increase when flight commences.
##COM_ARM_EKF_YAW
The COM_ARM_EKF_YAW parameter controls the maximum allowed inconsistency between the EKF inertial measurements and yaw measurement (magnetometer or external vision). The default value of 0.5 allows the differences to be no more than 50% of the maximum tolerated by the EKF and provides some margin for error increase when flight commences.
##COM_ARM_EKF_AB
The COM_ARM_EKF_AB parameter controls the maximum allowed EKF estimated IMU accelerometer bias. The default value of 0.005 allows for up to 0.5 m/s/s of accelerometer bias.
##COM_ARM_EKF_GB
The COM_ARM_EKF_GB parameter controls the maximum allowed EKF estimated IMU gyro bias. The default value of 0.00087 allows for up to 5 deg/sec of switch on gyro bias.
##COM_ARM_IMU_ACC
The COM_ARM_IMU_ACC parameter controls the maximum allowed inconsistency in acceleration measurements between the default IMU used for flight control and other IMU units if fitted. 
##COM_ARM_IMU_GYR
The COM_ARM_IMU_GYR parameter controls the maximum allowed inconsistency in angular rate measurements between the default IMU used for flight control and other IMU units if fitted.







