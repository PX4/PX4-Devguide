# VTOL Testing

How-to test that the VTOL functions properly, main focus are transitions:

  * On the bench
  * In flight

## General notes on transitions

There are currently 3 ways of commanding the VTOL to transition:

  * RC switch (2 pos, aux1)
  * MAVLink command (MAV_CMD_DO_VTOL_TRANSITION)
  * Transition during mission (MAV_CMD_DO_VTOL_TRANSITION internally)

When a transition is commanded (by either of the methods above), the VTOL enters the transition phase. If the VTOL receives a new transition command back to the old state during an ongoing transition it will switch back instantly. This is a safety feature to abort the transition when necessary. After the transition has been completed, the VTOL will be in the new state and a commanded transition into the reverse direction will take place normally.

> **Note** Make sure the AUX1 channel is assigned to an RC switch and that airspeed is working properly.

## On the bench

> **Caution** Remove all props! To test transition functionality properly, the vehicle needs to be armed.

By default, starting in multirotor mode:

  * arm the vehicle
  * check that motors are running in multirotor configuration (rudders/elevons should not move on roll/pitch/yaw inputs)
  * toggle transition switch
  * (if applicable) wait on step 1 of the transition phase to complete
  * blow into pito tube to simulate airspeed
  * (if applicable) step 2 of the transition phase will be executed
  * check that motors are running in fixed-wing configuration (roll/pitch/yaw inputs should control rudders/elevons)
  * toggle transition switch
  * observe back transition
  * check that motors are running in multirotor configuration (rudders/elevons should not move on roll/pitch/yaw inputs)

## In flight

> **Tip** Before testing transitions in flight, make sure the VTOL flies stable in multirotor mode. In general, if something doesn't go as planned, transition to multirotor mode and let it recover (it does a good job when it's properly tuned).

In-flight transition requires at least the following parameters to match your airframe and piloting skills:

| Param | Notes |
| :--- | :--- |
| VT_FW_PERM_STAB | Turns permanent stabilization on/off for fixed-wing. |
| VT_ARSP_BLEND | At which airspeed the fixed-wing controls becom active. |
| VT_ARSP_TRANS | At which airspeed the transition to fixed-wing is complete. |

There are more parameters depending on the type of VTOL, see the [parameter reference](https://pixhawk.org/firmware/parameters#vtol_attitude_control).

### Manual transition test

The basic procedure to test manual transitions is as follows:

  * arm and takeoff in multirotor mode
  * climb to a safe height to allow for some drop after transition
  * turn into the wind
  * toggle transition switch
  * observe transition **(MC-FW)**
  * fly in fixed-wing
  * come in at a safe height to allow for some drop after transition
  * toggle transition switch
  * observe transition **(FW-MC)**
  * land and disarm

**MC-FW**

During the transition from MC to FW the following can happen:

  1. it looses control while gaining speed (this can happen due to many factors)
  2. the transition takes too long and it flies too far away before the transition finishes

For 1): Switch back to multirotor (will happen instantly). Try to identify the problem (check setpoints).

For 2): If blending airspeed is set and it has a higher airspeed already it is controllable as fixed-wing. Therefore it is possible to fly around and give it more time to finish the transition. Otherwise switch back to multirotor and try to identify the problem (check airspeed).

**FW-MC**

The transition from FW to MC is mostly unproblematic. In-case it seems to loose control the best approach is to let it recover.

### Automatic transition test (mission, commanded)

Commanded transitions only work in auto (mission) or offboard flight-mode. Make sure you are confident to operate the auto/offboard and transition switch in flight.

Switching to manual will reactivate the transition switch. For example: if you switch out of auto/offboard when in automatic fixed-wing flight and the transition switch is currently in multirotor position it will transition to multirotor right away.

#### Proceduce

The following procedure can be used to test a mission with transition:

  * upload mission
  * takeoff in multirotor mode and climb to mission height
  * enable mission with switch
  * observe transition to fixed-wing flight
  * enjoy flight
  * observe transition back to multirotor mode
  * disable mission
  * land manually
  
During flight, the manual transition switch stays in multirotor position. If something doesn't go as planned, switch to manual and it will recover in multirotor mode.

#### Example mission

The mission should contain at least (also see screenshots below):

  * (1) position waypoint near takeoff location
  * (2) position waypoint in the direction of the planned fixed-wing flight route
  * (3) transition waypoint (to plane mode)
  * (4) position waypoint further away (at least as far away as the transition needs)
  * (6) position waypoint to fly back (a bit before takeoff location so back transition takes some distance)
  * (7) transition waypoint (to hover mode)
  * (8) position waypoint near takeoff location

![Mission, showing transition WP to plane](../../assets/vtol/qgc_mission_example_a.png)

![Mission, showing transition WP to hover](../../assets/vtol/qgc_mission_example_b.png)


