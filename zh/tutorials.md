The land detector is responsible for determining if the aircraft is still in air or landed. It is essentially a dynamic model of the vehicle which allows other parts of the system to configure the airframe correctly on-ground or in-air conditions.

The most reliable sensor to determine the land condition is a laser or radar altimeter. If the aircraft is not equipped with one the throttle configuration and velocity / accelerations can be used as a less reliable alternative.

## Multicopter Configuration

All multicopter land detection parameters start with `LNDMC` - not all of them are covered here, but they can be looked up in the QGroundControl parameter editor.

While most parameters are airframe agnostic, there are a few that are critical for reliable land detection in manual flight:

* Manual landed throttle: The throttle level at which the airframe is considered descending at maximum rate in manual mode. The parameter `LNDMC_MAN_DWNTHR` is set by default to 15%. A racer frame might require a lower setting, e.g. 8%.
* Hover throttle: The throttle value at which the airframe keeps its altitude. The parameter `MPC_THR_HOVER` is set by default to 50% throttle. It is important to correctly calibrate this.
* Manual takeoff throttle: The throttle level for takeoff. this should be above hover throttle, but not too far. The parameter `LNDMC_POS_UPTHR` is set to 65% throttle by default. Setting it closer to hover throttle will result in a more gentle takeoff.



## Fixed Wing Configuration

All fixed wing land detection parameters start with `LNDFW` - not all of them are covered here, but they can be looked up in the QGroundControl parameter editor.

* The maximum airspeed in landed state can be configured using the `LNDFW_AIRSPD_MAX` parameter - if the airspeed exceeds this value, the airframe is considered to be flying. It defaults to 8 m/s. Setting it much lower can result in bogus in-air detections as the accuracy of airspeed readings increases quadratically - around zero airspeed the noise is +- 4 m/s, at 20 m/s airspeed the noise is around 0.1 m/s.
* The maximum velocity in landed state can be configured using the `LNDFW_VELI_MAX` parameter. It represents the value of a short-term velocity integral. It is set by default to 4 m/s. As for airspeed lower values can lead to earlier in-air detection but increase false-positives for in-air detection.



