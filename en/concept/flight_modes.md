# Flight Modes

*Flight Modes* define how the autopilot responds to user input and controls vehicle movement. They are loosely grouped into *manual*, *assisted* and *auto* modes, based on the level/type of control provided by the autopilot. The pilot transitions between flight modes using switches on the remote control or with a [ground control station](../qgc/README.md).

Not all flight modes are available on all vehicle types, and some modes behave differently on different vehicle types (as described below). Finally, some flight modes make sense only under specific pre-flight and in-flight conditions (e.g. GPS lock, airspeed sensor, vehicle attitude sensing along an axis). The system will not allow transitions to those modes until the right conditions are met.

The sections below provide an overview of the modes, followed by a [flight mode evaluation diagram](#flight-mode-evaluation-diagram) that shows the conditions under which PX4 will transition into a new mode.


## Flight Mode Summary

### Manual flight modes

"Manual" modes are those where the user has direct control over the vehicle via the RC control (or joystick). Vehicle movement always follows stick movement, but the level/type of response changes depending on the mode. For example, experienced fliers can use modes that provide direct passthrough of stick positions to actuators, while beginners will often choose modes that are less responsive to sudden stick-position changes.

* **Fixed wing aircraft/ rovers / boats:** 
  * **MANUAL:** The pilot's control inputs are passed directly to the output mixer.
  * **STABILIZED:** The pilot's inputs are passed as roll and pitch *angle* commands and a manual yaw command.
* **Multirotors:**
  * **ACRO:** The pilot's inputs are passed as roll, pitch, and yaw *rate* commands to the autopilot.  This allows the multirotor to become completely inverted.  Throttle is passed directly to the output mixer
  * **RATTITUDE** The pilot's inputs are passed as roll, pitch, and yaw *rate* commands to the autopilot if they are greater than the mode's threshold.  If not the inputs are passed as roll and pitch  *angle* commands and a yaw *rate* command.  Throttle is passed directly to the output mixer.
  * **STABILIZED** The pilot's inputs are passed as roll and pitch *angle* commands and a yaw *rate* command.  Throttle is passed directly to the output mixer.
        
### Assisted flight modes

"Assisted" modes are also user controlled but offer some level of "automatic" assistance - for example, automatically holding position/direction, against wind. Assisted modes often make it much easier to gain or restore controlled flight.

* **ALTCTL** (Altitude Control)
  * **Fixed wing aircraft:** When the roll, pitch and yaw inputs (RPY) are all centered (less than some specified deadband range) the aircraft will return to straight and level flight and keep its current altitude. It will drift with the wind.
  * **Multirotors:** Roll, pitch and yaw inputs are as in MANUAL mode. Throttle inputs indicate climb or sink at a predetermined maximum rate. Throttle has large deadzone.
* **POSCTL** (Position Control)
  * **Fixed wing aircraft:** Neutral inputs give level, flight and it will crab against the wind if needed to maintain a straight line.
  * **Multirotors** Roll controls left-right speed, pitch controls front-back speed over ground. When roll and pitch are all centered (inside deadzone) the multirotor will hold position. Yaw controls yaw rate as in MANUAL mode. Throttle controls climb/descent rate as in ALTCTL mode.
      
### Auto flight modes

"Auto" modes are those where the controller requires little to no user input (e.g. to takeoff, land and fly missions).

* **AUTO_LOITER** (Loiter)
  * **Fixed wing aircraft:** The aircraft loiters around the current position at the current altitude (or possibly slightly above the current altitude, good for 'I'm losing it'). 
  * **Multirotors:**  The multirotor hovers / loiters at the current position and altitude.
* **AUTO_RTL** (Return to Land)
  * **Fixed wing aircraft:** The aircraft returns to the home position and loiters in a circle above the home position. 
  * **Multirotors:** The multirotor returns in a straight line on the current altitude (if higher than the home position + loiter altitude) or on the loiter altitude (if higher than the current altitude), then lands automatically.
* **AUTO_MISSION** (Mission)
  * **All system types:** The aircraft obeys the programmed mission sent by the ground control station (GCS). If no mission received, aircraft will LOITER at current position instead.
  * **_OFFBOARD_** (Offboard)
    In this mode the position, velocity or attitude reference / target / setpoint is provided by a companion computer connected via serial cable and MAVLink. The offboard setpoint can be provided by APIs like [MAVROS](https://github.com/mavlink/mavros) or [Dronekit](http://dronekit.io).

## Flight Mode Evaluation Diagram

![](../../assets/diagrams/commander-flow-diagram.png)
