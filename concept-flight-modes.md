# Flight Modes

**Flight Modes** define the state of the system at any given time.  The user transitions between flight modes via switches on the remote control or the [ground control station](qgroundcontrol-intro.md).

## Flight Mode Quick Summary

  * **_MANUAL_**
    * **Fixed wing aircraft/ rovers / boats:** 
        * **MANUAL:** The pilot's control inputs are passed directly to the output mixer.
        * **STABILIZED:** The pilot's inputs are passed as roll and pitch *angle* commands and a manual yaw command.
    * **Multirotors:**
        * **ACRO:** The pilot's inputs are passed as roll, pitch, and yaw *rate* commands to the autopilot.  This allows the multirotor to become completely inverted.  Throttle is passed directly to the output mixer
        * **RATTITUDE** The pilot's inputs are passed as roll, pitch, and yaw *rate* commands to the autopilot if they are greater than the mode's threshold.  If not the inputs are passed as roll and pitch  *angle* commands and a yaw *rate* command.  Throttle is passed directly to the output mixer.
        * **ANGLE** The pilot's inputs are passed as roll and pitch *angle* commands and a yaw *rate* command.  Throttle is passed directly to the output mixer.
  * **_ASSISTED_**
    * **ALTCTL**
      * **Fixed wing aircraft:** When the roll, pitch and yaw inputs (RPY) are all centered (less than some specified deadband range) the aircraft will return to straight and level flight and keep its current altitude. It will drift with the wind.
      * **Multirotors:** Roll, pitch and yaw inputs are as in MANUAL mode. Throttle inputs indicate climb or sink at a predetermined maximum rate. Throttle has large deadzone.
    * **POSCTL**
      * **Fixed wing aircraft:** Neutral inputs give level, flight and it will crab against the wind if needed to maintain a straight line.
      * **Multirotors** Roll controls left-right speed, pitch controls front-back speed over ground. When roll and pitch are all centered (inside deadzone) the multirotor will hold position. Yaw controls yaw rate as in MANUAL mode. Throttle controls climb/descent rate as in ALTCTL mode.
  * **_AUTO_**
    * **AUTO_LOITER**
        * **Fixed wing aircraft:** The aircraft loiters around the current position at the current altitude (or possibly slightly above the current altitude, good for 'I'm losing it'). 
        * **Multirotors:**  The multirotor hovers / loiters at the current position and altitude.
    * **AUTO_RTL**
        * **Fixed wing aircraft:** The aircraft returns to the home position and loiters in a circle above the home position. 
        * **Multirotors:** The multirotor returns in a straight line on the current altitude (if higher than the home position + loiter altitude) or on the loiter altitude (if higher than the current altitude), then lands automatically.
    * **AUTO_MISSION**
        * **All system types:** The aircraft obeys the programmed mission sent by the ground control station (GCS). If no mission received, aircraft will LOITER at current position instead.
  * **_OFFBOARD_**
    In this mode the position, velocity or attitude reference / target / setpoint is provided by a companion computer connected via serial cable and MAVLink. The offboard setpoint can be provided by APIs like [MAVROS](https://github.com/mavlink/mavros) or [Dronekit](http://dronekit.io).

## Flight Mode Evaluation Diagram
![](images/diagrams/commander-flow-diagram.png)
