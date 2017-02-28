# EKF2 Log Replay

This page shows you how you can tune the parameters of the EKF2 estimator by using the replay feature on a real flight log.

## Introduction
A developer has the possibility to do replay on particularly logged data for estimation analysis. The remainder of this page will explain which parameters have to be set in order to benefit from this feature and how to correctly deploy it.

## sdlog2 logger (.px4log)

### Prerequisites

* set the parameter **EKF2\_REC\_RPL** to 1. This will tell the estimator to publish **special** replay messages for logging. 
* set the parameter **SDLOG\_PRIO\_BOOST** to a value contained in the set {0, 1, 2, 3}. A value of 0 means that the onboard logging app has a default \(low\) scheduling priority. A low scheduling priority can lead to a loss of logging messages. If you find that your log file contains 'gaps' due to skipped messages then you can increase this parameter to a maximum value of 3. Testing has shown that a minimum value of 2 is required in order to avoid loss of data.

### Deployment

Once you have a real flight log (.px4log) created with the above settings then you can **run** a replay on it by using the following command in the root directory of your PX4 Firmware:

```
make posix_sitl_replay replay logfile=absolute_path_to_log_file/my_log_file.px4log
```

where 'absolute\_path\_to\_log\_file/my\_log\_file.px4log' is a placeholder for the absolute path of the log file you want to run the replay on. Once the command has executed check the terminal for the location and name of the replayed log file. 
This file should be located in
```
<path to Firmware>/build_posix_sitl_replay/src/firmware/posix/rootfs/
```
The output replayed file is called **replay_replayed.px4log**, which can be used to analyze the estimator performance. 


### Changing tuning parameters for a replay
When you run the replay the first time, a replay_replayed.px4log file will be generated using the default EKF2 parameter values from the actual flight. 
After that you can change any EKF2 parameter value by changing the estimator parameters in **replay\_params.txt**, which is located in the same directory as your replay_replayed.px4log output file.

For example setting the noise value for the gyro bias would require the following line.

```
EKF2_GB_NOISE 0.001
```
Once EKF2 parameter is changed, a new replay_replay.px4log file can be gnerated using the same command given in [Deployment](deployment).

## ulog logger
Is coming...
