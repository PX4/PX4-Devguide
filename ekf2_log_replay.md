# EKF2 Log Replay

This page shows you how you can tune the parameters of the EKF2 estimator by using the replay feature on a real flight log. It is based on the `sdlog2` logger.

## Introduction

A developer has the possibility to enable onboard logging of the EKF2 estimator input sensor data. This allows him to tune the parameters of the estimator offline by running a replay on the logged data trying out different sets of parameters. The remainder of this page will explain which parameters have to be set in order to benefit from this feature and how to correctly deploy it.

## Prerequisites

* set the parameter **EKF2\_REC\_RPL** to 1. This will tell the estimator to publish special replay messages for logging.
* set the parameter **SDLOG\_PRIO\_BOOST** to a value contained in the set {0, 1, 2, 3}. A value of 0 means that the onboard logging app has a default \(low\) scheduling priority. A low scheduling priority can lead to a loss of logging messages. If you find that your log file contains 'gaps' due to skipped messages then you can increase this parameter to a maximum value of 3. Testing has shown that a minimum value of 2 is required in order to avoid loss of data.

## Deployment

Once you have a real flight log then you can run a replay on it by using the following command in the root directory of your PX4 Firmware

```
make posix_sitl_replay replay logfile=absolute_path_to_log_file/my_log_file.px4log
```

where 'absolute\_path\_to\_log\_file/my\_log\_file.px4log' is a placeholder for the absolute path of the log file you want to run the replay on. Once the command has executed check the terminal for the location and name of the replayed log file.

## Changing tuning parameters for a replay

You can set the estimator parameter values for the replay in the file **replay\_params.txt** located in the same directory as your replayed log file, e.g. **build\_posix\_sitl\_replay/src/firmware/posix/rootfs/replay\_params.txt**. When running the replay for the first time \(e.g. after a **make clean**\) this file will be auto generated and filled with the default EKF2 parameter values taken from the flight log. After that you can change any EKF2 parameter value by changing the corresponding number in the text file. Setting the noise value for the gyro bias would require the following line.

```
EKF2_GB_NOISE 0.001
```
