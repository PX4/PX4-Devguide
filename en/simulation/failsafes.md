# Simulate Failsafes

It's often useful to test safety critical behavior in simulation before taking any risk with a real world scenario. While in HITL simulation the configuration parameters of your real flight control unit is used, in SITL simulation most failsafes are disabled by default to enable quick and easy simulation usage. Also for example the virtual battery is implemented to never run out of energy.

## Data Link Loss
By default unavailability of external data via MAVLink is ignored to make the simulation usable without dependency on ground control station, attached SDK or offboard instance.

To test data link loss reactions set the parameter `NAV_DLL_ACT` to the desired reaction according to the possibilities listed in the [parameter reference](advanced/parameter_reference.html) e.g. to the value `2` to make the vehicle return to launch.

> **Note** This parameter gets reset once SITL is restarted. To prevent that from happening remove the line `param set NAV_DLL_ACT 0` from the [SITL startup script](https://github.com/PX4/Firmware/blob/1d86b7570a75844375cb4c7d75731ec33826f173/ROMFS/px4fmu_common/init.d-posix/rcS).

## RC Link Loss
By default unavailability of any remote remote control (RC) data is ignored to make the simulation conveniently usable without additonal dependencies.

To test RC loss set the parameter `NAV_RCL_ACT` to the desired reaction according to the possibilities listed in the [parameter reference](advanced/parameter_reference.html) e.g. to the value `2` to make the vehicle return to launch.

> **Note** This parameter gets reset once SITL is restarted. To prevent that from happening remove the line `param set NAV_RCL_ACT 0` from the [SITL startup script](https://github.com/PX4/Firmware/blob/1d86b7570a75844375cb4c7d75731ec33826f173/ROMFS/px4fmu_common/init.d-posix/rcS).

## Low Battery
By default the simulated battery only depletes to 50% of its capacity to enable testing of any battery indication but not triggering any low battery reactions that might interrupt ongoing simulation testing.
To change this minimal percentage value change [this line](https://github.com/PX4/Firmware/blob/9d67bbc328553bbd0891ffb8e73b8112bca33fcc/src/modules/simulator/simulator_mavlink.cpp#L330).

To control how fast the battery depletes to this value use the parameter `SIM_BAT_DRAIN`. By changing this configuration in flight you can also test regaining capacity to simulate inaccurate battery state estimation or in air charging technology.

## GPS Loss
To make the simulation virtually loose position information even though it's always available you can just stop the GPS driver by running the command `gpssim stop` on your SITL instances pxh shell. If you want to recover GPS run `gpssim start` again.
