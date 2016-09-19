# Gimbal Control Setup

PX4 contains a generic mount/gimbal control driver with different input and output methods. Any input can be selected to drive any output.

First, make sure the driver runs, using `vmount start`, then configure its parameters.

## Parameters
The parameters are described in [src/drivers/vmount/vmount_params.c](https://github.com/PX4/Firmware/blob/master/src/drivers/vmount/vmount_params.c). The most important ones are the input (`MNT_MODE_IN`) and the output (`MNT_MODE_OUT`) mode. By default, the input is disabled. Any input method can be selected to drive any of the available outputs.

If a mavlink input mode is selected, manual RC input can be enabled in
addition (`MNT_MAN_CONTROL`). It is active as long as no mavlink message is received yet, or mavlink explicitly requests RC mode.



### Configure the gimbal mixer for AUX output

The gimbal uses the control group #2 (see [Mixing and Actuators](concept-mixing.md)). This is the mixer configuration:

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
```

Add those you need to your main or auxiliary mixer.

## Testing
The driver provides a simple test command - it needs to be stopped first with `vmount stop`. The following describes testing in SITL, but the commands also work on a real device.

Start the simulation with (no parameter needs to be changed for that):
```
make posix gazebo_typhoon_h480
```
Make sure it's armed, eg. with `commander takeoff`, then use for example
```
vmount test yaw 30
```
to control the gimbal. Note that the simulated gimbal stabilizes itself, so if you send mavlink commands, set the `stabilize` flags to false.

![Gazebo Gimbal Simulation](images/gazebo-gimbal-simulation.png)

