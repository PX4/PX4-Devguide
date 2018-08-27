# Multi-Vehicle Simulation with JMAVSim

This topic explains how to simulate multiple UAV (multicopter) vehicles using JMAVSim and SITL.

> **Tip** This is the easiest way to simulate multiple vehicles running PX4. 
  It works on PX4 v1.8.0 and later.

## Instances on Separate Ports

To start multiple instances on separate ports:

1. Build PX4
   ```
   make posix_sitl_default
   ```
1. Run **sitl_multiple_run.sh**, specifying the number of instances to start (e.g. 2):
   ```
   ./Tools/sitl_multiple_run.sh 2
   ```
1. Start the first instance:
   ```
   ./Tools/jmavsim_run.sh
  ```
1. Start subsequent instances, specifying the UDP port for the instance:
   ```
   ./Tools/jmavsim_run.sh -p 14561
   ```
   The port should be set to `14560+i` for `i` in `[0, N-1]`.

## Instances on Same Port

To have multiple (two in this case) instances of PX4 connect on the *same* port, you can modify **init.d-posix/rcS** as shown:
```
diff --git a/ROMFS/px4fmu_common/init.d-posix/rcS b/ROMFS/px4fmu_common/init.d-posix/rcS
index 91c9cb7c8f..f276357d90 100644
--- a/ROMFS/px4fmu_common/init.d-posix/rcS
+++ b/ROMFS/px4fmu_common/init.d-posix/rcS
@@ -83,7 +83,7 @@ fi
 param set MAV_SYS_ID $((1+px4_instance))
 simulator_udp_port=$((14560+px4_instance))
 udp_offboard_port_local=$((14557+px4_instance))
-udp_offboard_port_remote=$((14540+px4_instance))
+udp_offboard_port_remote=$((14540))
 udp_gcs_port_local=$((14556+px4_instance))
 
 if [ $AUTOCNF == yes ]
```


## Additional Resources

* See [Simulation](../simulation/README.md) for more information about the UDP port configuration.
