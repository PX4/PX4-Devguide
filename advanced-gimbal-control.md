# Gimbal Control Setup

This is a rough scrapbook on how to setup gimbal control via MAVLink.

## Relevant parameters

GMB_USE_MNT

GMB_AUX_MNT_CHN

## MAVLink control commands

Use [COMMAND_LONG](https://pixhawk.ethz.ch/mavlink/#COMMAND_LONG) with the following enums:

[MAV_CMD_DO_MOUNT_CONTROL](https://pixhawk.ethz.ch/mavlink/#MAV_CMD_DO_MOUNT_CONTROL) to control the outputs directly.

[MAV_CMD_DO_MOUNT_CONFIGURE](https://pixhawk.ethz.ch/mavlink/#MAV_CMD_DO_MOUNT_CONFIGURE) to configure the gimbal app.

## Setup

The setup requires knowledge about how to configure the [system startup](advanced-system-startup.html#customizing-the-system-startup).

### Start gimbal app

Add the following command to your boot process:

```
gimbal start
```

### Configure the gimbal mixer

These are the supported outputs:

```
# roll
M: 1
O:      10000  10000      0 -10000  10000
S: 2 0  10000  10000      0 -10000  10000

# pitch
M: 1
O:      10000  10000      0 -10000  10000
S: 2 1  10000  10000      0 -10000  10000

# yaw
M: 1
O:      10000  10000      0 -10000  10000
S: 2 2  10000  10000      0 -10000  10000

# shutter (currently not implemented by the gimbal app)
#M: 1
#O:      10000  10000      0 -10000  10000
#S: 2 3  10000  10000      0 -10000  10000

# mount, retractables
M: 1
O:      10000  10000      0 -10000  10000
S: 2 4  10000  10000      0 -10000  10000
```

Add those you need to your main or auxiliary mixer.

