# Hardware in the Loop Simulation (HITL)

Hardware in the loop simulation is a simulation mode where the autopilot is connected to the simulator and all flight code runs on the autopilot. This approach has the benefit of testing the actual flight code on the real processor.

## Configure the System for HITL

PX4 supports HITL for multicopters (using jMAVSim) and fixed wing (using X-Plane demo or full). Flightgear support is present as well, but we generally recommend X-Plane. To enable it, configure it via the airframe menu.

![](images/gcs/qgc_hil_config.png)

## Switch to Joystick Input

If a joystick is preferred over a radio remote control, set the parameter `COM_RC_IN_MODE` to `1`. It can be found in the Commander parameter group.

## Enable Remote Access in X-Plane

In X-Plane two key settings have to be made: In Settings -> Data Input and Output, set these checkboxes:

![](images/gcs/xplane_data_config.png)

In Settings -> Net Connections in the Data tab, set localhost and port 49005 as IP address, as shown in the screenshot below:

![](images/gcs/xplane_net_config.png)

## Enable HITL in QGroundControl

Widgets -> HIL Config, then select X-Plane 10 in the drop-down and hit connect. Once the system is connected, battery status, GPS status and aircraft position should all become valid:

![](images/gcs/qgc_sim_run.png)

## Fly an Autonomous Mission in HITL

Switch to the flight planning view and put a single waypoint in front of the plane. Click on the sync icon to send the waypoint.

Then select MISSION from the flight mode menu in the toolbar and click on DISARMED to arm the plane. It will take off and loiter around the takeoff waypoint.

![](images/gcs/qgc_sim_mission.png)
