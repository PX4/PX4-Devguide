# Land Detector Configuration

The land detector is a dynamic vehicle model representing key vehicle states such as landed and ground contact.

## Configuring Auto-Disarming

By default the land detector does detect landing, but does not auto-disarm. If the hysteresis parameter [COM_DISARM_LAND](../advanced/parameter_reference.md#COM_DISARM_LAND) is set to a non-zero value the system will auto-disarm after N seconds \(the value it is set to\).

## Multicopter Land Detector Configuration

The complete set of parameters is available in the *QGroundControl* parameter editor under the `LNDMC` prefix. The key parameters that might differ per airframe are these:

* [MPC_THR_HOVER](../advanced/parameter_reference.md#MPC_THR_HOVER) - the hover throttle of the system \(in percent, default is 50%\). It is important to set this correctly as not only it does it make the altitude control more accurate, but it also ensures correct land detection. A racer or a big camera drone without payload mounted might need a much lower setting \(e.g. 35%\).
* [MPC_THR_MIN](../advanced/parameter_reference.md#MPC_THR_MIN) - the overall minimum throttle of the system. This should be set to enable a controlled descent.
* [LNDMC_THR_RANGE](../advanced/parameter_reference.md#LNDMC_THR_RANGE) - this is a scaling factor to define the range between min and hover throttle that gets accepted as landed. 
  Example: If the minimum throttle is 0.1, the hover throttle is 0.5 and the range is 0.2 \(20%\), then the highest throttle value that counts as landed is: `0.1 + (0.5 - 0.1) * 0.2 = 0.18`.


### Land detector states

In order to detect landing, the multicopter first has to go through three different states, where each state contains the conditions from the previous states plus tighter constraints. 
If a condition cannot be reached because of missing sensors, then the condition is true by default. 
For instance, in Acro mode and no sensor is active except for the gyro sensor, then the detection solely relies on thrust output and time. 
 
In order to proceed to the next state, each condition has to be true for some predefined time. 
If one condition fails, the land detector drops out of the current state immediately. 

#### Ground contact

This state is reached if following conditions are true for 0.35 seconds:

- no vertical movement ([LNDMC_Z_VEL_MAX](../advanced/parameter_reference.md#LNDMC_Z_VEL_MAX))
- no horizontal movement ([LNDMC_XY_VEL_MAX](../advanced/parameter_reference.md#LNDMC_XY_VEL_MAX))
- low thrust `MPC_THR_MIN + (MPC_THR_HOVER - MPC_THR_MIN) * 0.3f` or velocity setpoint is 0.9 of land speed but vehicle has no vertical movement.

If the vehicle is in position or velocity control and ground contact were detected, 
the position controller will set the thrust vector along the body x-y-axis to zero.


#### Maybe landed

This state is reached if following conditions are true for 0.25 seconds:

- all conditions of ground contact are true
- is not rotating ([LNDMC_ROT_MAX](../advanced/parameter_reference.md#LNDMC_ROT_MAX))
- has low thrust `MPC_THR_MIN + (MPC_THR_HOVER - MPC_THR_MIN) * LNDMC_THR_RANGE`; 

If the vehicle only has knowledge of thrust and angular rate, 
in order to proceed to the next state the vehicle has to have low thrust and no rotation for 8.0 seconds. 

If the vehicle is in position or velocity control and maybe landed was detected, the position controller will set the thrust vector to zero. 


#### Landed

This state is reached if following conditions are true for 0.3 seconds:
- all conditions of maybe landed are true


## Fixed Wing Land Detector Configuration

The complete set of parameters is available under the `LNDFW` prefix. These two user parameters are sometimes worth tuning:

* [LNDFW_AIRSPD_MAX](../advanced/parameter_reference.md#LNDFW_AIRSPD_MAX) - the maximum airspeed allowed for the system still to be considered landed. The default of 8 m/s is a reliable tradeoff between airspeed sensing accuracy and triggering fast enough. Better airspeed sensors should allow lower values of this parameter.
* [LNDFW_VELI_MAX](../advanced/parameter_reference.md#LNDFW_VELI_MAX) - the maximum velocity for the system to be still considered landed. This parameter can be adjusted to ensure a sooner or later land detection on throwing the airframe for hand-launches.



