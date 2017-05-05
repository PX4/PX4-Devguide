# Modules Reference: System
## logger
Source: [modules/logger](https://github.com/PX4/Firmware/tree/master/src/modules/logger)


### Description
System logger which logs a configurable set of uORB topics and system printf messages
(`PX4_WARN` and `PX4_ERR`) to ULog files. These can be used for system and flight performance evaluation,
tuning, replay and crash analysis.

It supports 2 backends:
- Files: write ULog files to the file system (SD card)
- MAVLink: stream ULog data via MAVLink to a client (the client must support this)

Both backends can be enabled and used at the same time.

### Implementation
The implementation uses two threads:
- The main thread, running at a fixed rate (or polling on a topic if started with -p) and checking for
  data updates
- The writer thread, writing data to the file

In between there is a write buffer with configurable size. It should be large to avoid dropouts.

### Examples
Typical usage to start logging immediately:
```
logger start -e -t
```

Or if already running:
```
logger on
```

### Usage
```
logger <command> [arguments...]
 Commands:
   start
     [-m <val>]  Backend mode
                 values: file|mavlink|all, default: all
     [-e]        Enable logging right after start until disarm (otherwise only
                 when armed)
     [-f]        Log until shutdown (implies -e)
     [-t]        Use date/time for naming log directories and files
     [-r <val>]  Log rate in Hz, 0 means unlimited rate
                 default: 280
     [-b <val>]  Log buffer size in KiB
                 default: 12
     [-q <val>]  uORB queue size for mavlink mode
                 default: 14
     [-p <val>]  Poll on a topic instead of running with fixed rate (Log rate
                 and topic intervals are ignored if this is set)
                 values: <topic_name>

   on            start logging now, override arming (logger must be running)

   off           stop logging now, override arming (logger must be running)

   stop

   status        print status info
```
## send_event
Source: [modules/events](https://github.com/PX4/Firmware/tree/master/src/modules/events)


### Description
Background process running periodically on the LP work queue to perform housekeeping tasks.
It is currently only responsible for temperature calibration.

The tasks can be started via CLI or uORB topics (vehicle_command from MAVLink, etc.).

### Usage
```
send_event <command> [arguments...]
 Commands:
   start         Start the background task

   temperature_calibration Run temperature calibration process
     [-g]        calibrate the gyro
     [-a]        calibrate the accel
     [-b]        calibrate the baro (if none of these is given, all will be
                 calibrated)

   stop

   status        print status info
```
