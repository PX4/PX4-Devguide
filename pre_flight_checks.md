# Preflight Checks
The commander module performs a number of preflight checks which are controlled by the the COM_ARM<> parameters. If these checks fail, the motors are prevented from arming.

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
The COM_ARM_EKF_AB parameter controls the maximum allowed EKF estimated IMU gyro bias. The default value of 0.00087 allows for up to 5 deg/sec of switch on gyro bias.





