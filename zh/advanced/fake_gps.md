# FAKE GPS
This page shows you how to use mocap data to fake gps. 

The setup looks as follows:
There is a "VICON computer" which has the required software installed + [ROS](http://www.ros.org/) + [vicon_bridge](https://github.com/ethz-asl/vicon_bridge) and sends this data over network to "your Computer".
On "your computer" you should have ROS + MAVROS installed. In MAVROS there is a script which simulates gps data out of mocap data.
"Your computer" then sends the data over 3DR radiometry to the pixhawk.
*NOTE*: The "VICON computer" and "your computer" can be the same, of course.

## Prerequisites
* MOCAP system (in this example, VICON is used)
* Computer with ROS, MAVROS and Vicon_bridge
* 3DR radiometry set

## Procedure
### Step 1
Make sure "your computer" is in the same network as the "VICON computer" (maybe you need a wireless adapter).
Create two files on the "VICON computer": "launch_fake_gps.sh" and "launch_fake_gps_distorted.sh" 

Add the following two lines to the "launch_fake_gps.sh" file and replace xxx.xxx.x.xxx with the IP adress of "your computer" (you can get the IP adress by typing "ifconfig" in the terminal).
```sh
export ROS_MASTER_URI=http://xxx.xxx.x.xxx:11311
roslaunch vicon_bridge vicon.launch $@
```

Next, add the following two lines to the "launch_fake_gps_distorted.sh" file and replace xxx.xxx.x.xxx with the IP adress of "your computer".
```sh
export ROS_MASTER_URI=http://xxx.xxx.x.xxx:11311
rosrun vicon_bridge tf_distort $@
```

Put markers on your drone and create a model in the MOCAP system (later referred to as yourModelName).

### Step 2
Run
```sh
$ roscore
```
on "your computer".


### Step 3
Run
```sh
$ sh launch_fake_gps.sh
```
and
```sh
$ sh launch_fake_gps_distorted.sh
```
in two different terminals on the "VICON computer" in the directory where you created the two files.


### Step 4
On "your computer" run
```sh
$ rosrun rqt_reconfigure rqt_reconfigure
```
A new window should open where you can select "tf_distort". With this tool you can edit the parameters to distort the MOCAP data.

To simulate GPS we use:
* publish rate = 5.0Hz
* tf_frame_in = vicon/yourModelName/yourModelName (e.g. vicon/DJI_450/DJI_450)
* delay = 200ms
* sigma_xy = 0.05m
* sigma_z = 0.05m


### Step 5
Connect your pixhawk in QGroundControl. Go to PARAMETERS -> System and change SYS_COMPANION to 257600 (enables magic mode;)).

Next, go to PARAMETERS -> MAVLink and change MAV_USEHILGPS to 1 (enable HIL GPS).

Now, go to PARAMETERS -> Attitude Q estimator and change ATT_EXT_HDG_M to 2 (use heading from motion capture).

Last but not least, go to PARAMETERS -> Position Estimator INAV and change INAV_DISAB_MOCAP to 1 (disable mocap estimation).

*NOTE*: if you can't find the above stated parameters, check PARAMETERS -> default Group


### Step 6
Next, open "mocap_fake_gps.cpp". You should find it at: yourCatkinWS/src/mavros/mavros_extras/src/plugins/mocap_fake_gps.cpp

Replace DJI_450/DJI_450 in
```sh
mocap_tf_sub = mp_nh.subscribe("/vicon/DJI_450/DJI_450_drop", 1, &MocapFakeGPSPlugin::mocap_tf_cb, this);
```
with your model name (e.g. /vicon/yourModelName/yourModelname_drop). The "_drop" will be explained in the next step.


### Step 7
In step 5, we enabled heading from motion capture. Therefore pixhawk does not use the original North, East direction, but the one from the motion capture system. Because the 3DR radiometry device is not fast enought, we have to limit the rate of our MOCAP data. To do this run
```sh
$ rosrun topic_tools drop /vicon/yourModelName/yourModelName 9 10
```
This means, that from the rostopic /vicon/yourModelName/yourModelName, 9 out of 10 messages will be droped and published under the topic name "/vicon/yourModelName/yourModelName_drop".


### Step 8
Connect the 3DR radiometry with the pixhawk TELEM2 and the counter part with your computer (USB).


### Step 9
Go to your catkinWS and run
```sh
$ catkin build
```
and afterwards
```sh
$ roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB0:57600
```
That's it! Your pixhawk now gets GPS data and the light should pulse in green color.
