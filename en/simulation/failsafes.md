# Simulate Failsafes

[Failsafes](https://docs.px4.io/en/config/safety.html) define the safe limits/conditions under which you can safely use PX4, and the action that will be performed if a failsafe is triggered (for example, landing, holding position, or returning to a specified point).

In SITL most failsafes are disabled by default to enable easier simulation usage.
This topic explains how you can test safety-critical behavior in SITL simulation before attempting it in the real world.

> **Note** You can also test failsafes using [HITL simulation](../simulation/hitl.md). 
  HITL uses the normal configuration parameters of your flight controller.


## Data Link Loss

The *Data Link Loss* failsafe (unavailability of external data via MAVLink) is ignored by default.
This makes the simulation usable without a connected GCS, SDK, or other MAVLink application.

Set the parameter [NAV_DLL_ACT](../advanced/parameter_reference.md#NAV_DLL_ACT) to the desired failsafe action to enable the failsafe. 
For example, set to `2` to make the vehicle return to launch.

> **Note** The parameter gets reset if SITL is restarted. 
  To prevent that from happening remove the line `param set NAV_DLL_ACT 0` from the [SITL startup script](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d-posix/rcS).

## RC Link Loss

The *RC Link Loss* failsafe (unavailability of data from a remote control) is ignored by default.
This makes the simulation usable without additional hardware.

Set the parameter [NAV_RCL_ACT](../advanced/parameter_reference.md#NAV_RCL_ACT) to the desired failsafe action to enable the failsafe. 
For example, set to `2` to make the vehicle return to launch.

> **Note** The parameter gets reset if SITL is restarted. 
  To prevent that from happening remove the line `param set NAV_RCL_ACT 0` from the [SITL startup script](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d-posix/rcS).


## Low Battery

The simulated battery is implemented to never run out of energy, and by default only depletes to 50% of its capacity. 
This enables testing of battery indication in GCS UIs without triggering low battery reactions that might interrupt other testing.

To change this minimal battery percentage value change [this line](https://github.com/PX4/Firmware/blob/9d67bbc328553bbd0891ffb8e73b8112bca33fcc/src/modules/simulator/simulator_mavlink.cpp#L330).

To control how fast the battery depletes to the minimal value use the parameter [SIM_BAT_DRAIN](../advanced/parameter_reference.md#SIM_BAT_DRAIN).

> **Tip** By changing this configuration in flight, you can also test regaining capacity to simulate inaccurate battery state estimation or in-air charging technology.

## GPS Loss

To make simulate losing and regaining GPS information you can just stop/restart the GPS driver. 
This is done by running the `gpssim stop` and `gpssim start` commands on your SITL instance *pxh shell*.
