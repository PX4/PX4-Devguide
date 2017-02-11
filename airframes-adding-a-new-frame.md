# Adding a new Airframe Configuration

PX4 uses canned configurations as starting point for airframes. Adding a configuration is straightforward: Create a new file which is prepended with a free autostart ID in the [init.d folder](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d) and [build and upload](starting-building.md) the software.

Developers not wanting to create their own configuration can instead customize existing configurations using text files on the microSD card, as detailed on the [custom system startup](advanced-system-startup.md) page.

## Airframe configurations

An airframe configuration consists of three main blocks:

  * The apps it should start, e.g. multicopter or fixed wing controllers
  * The physical configuration of the system (e.g. a plane, wing or multicopter). This is called mixer.
  * Tuning gains

These three aspects are mostly independent, which means that many configurations share the same physical layout of the airframe and start the same applications and most differ in their tuning gains.

All configurations are stored in the [ROMFS/px4fmu_common/init.d](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d) folder. All mixers are stored in the [ROMFS/px4fmu_common/mixers](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/mixers) folder.

### Config file

A typical configuration file is below.

```bash
#!nsh
#
# @name Wing Wing (aka Z-84) Flying Wing
#
# @url https://pixhawk.org/platforms/planes/z-84_wing_wing
#
# @type Flying Wing
#
# @output MAIN1 left aileron
# @output MAIN2 right aileron
# @output MAIN4 throttle
#
# @output AUX1 feed-through of RC AUX1 channel
# @output AUX2 feed-through of RC AUX2 channel
# @output AUX3 feed-through of RC AUX3 channel
#
# @maintainer Lorenz Meier <lorenz@px4.io>
#

sh /etc/init.d/rc.fw_defaults

if [ $AUTOCNF == yes ]
then
	param set BAT_N_CELLS 2
	param set FW_AIRSPD_MAX 15
	param set FW_AIRSPD_MIN 10
	param set FW_AIRSPD_TRIM 13
	param set FW_ATT_TC 0.3
	param set FW_L1_DAMPING 0.74
	param set FW_L1_PERIOD 16
	param set FW_LND_ANG 15
	param set FW_LND_FLALT 5
	param set FW_LND_HHDIST 15
	param set FW_LND_HVIRT 13
	param set FW_LND_TLALT 5
	param set FW_THR_LND_MAX 0
	param set FW_PR_FF 0.35
	param set FW_RR_FF 0.6
	param set FW_RR_P 0.04
fi

# Configure this as plane
set MAV_TYPE 1
# Set mixer
set MIXER wingwing
# Provide ESC a constant 1000 us pulse
set PWM_OUT 4
set PWM_DISARMED 1000
```

IMPORTANT REMARK: If you want to reverse a channel, never do this neither on your RC transmitter nor with e.g `RC1_REV`. The channels are only reversed when flying in manual mode, when you switch in an autopilot flight mode, the channels output will still be wrong (it only inverts your RC signal). Thus for a correct channel assignment change either your PWM signals with `PWM_MAIN_REV1` (e.g. for channel one) or change the signs for both output scaling and output range in the corresponding mixer (see below). 

### Mixer file

A typical configuration file is below. 

<aside class="note">
The plugs of the servos / motors go in the order of the mixers in this file.
</aside>

So MAIN1 would be the left aileron, MAIN2 the right aileron, MAIN3 is empty (note the Z: zero mixer) and MAIN4 is throttle (to keep throttle on output 4 for common fixed wing configurations).

A mixer is encoded in normalized units from -10000 to 10000, corresponding to -1..+1.

```
M: 2
O:      10000  10000      0 -10000  10000
S: 0 0  -6000  -6000      0 -10000  10000
S: 0 1   6500   6500      0 -10000  10000
```

Where each number from left to right means:

  * M: Indicates two scalers for two inputs
  * O: Indicates the output scaling (*1 in negative, *1 in positive), offset (zero here), and output range (-1..+1 here).  If you want to invert your PWM signal, the signs for both output scalings and both output range numbers have to be changed. (```O:      -10000  -10000      0 10000  -10000```)
  * S: Indicates the first input scaler: It takes input from control group #0 (attitude controls) and the first input (roll). It scales the input * 0.6 and reverts the sign (-0.6 becomes -6000 in scaled units). It applies no offset (0) and outputs to the full range (-1..+1)
  * S: Indicates the second input scaler: It takes input from control group #0 (attitude controls) and the second input (pitch). It scales the input * 0.65 and reverts the sign (-0.65 becomes -6500 in scaled units). It applies no offset (0) and outputs to the full range (-1..+1)

Both scalers are added, which for a flying wing means the control surface takes maximum 60% deflection from roll and 65% deflection from pitch. As it is over-committed with 125% total deflection for maximum pitch and roll, it means the first channel (roll here) has priority over the second channel / scaler (pitch).

The complete mixer looks like this:


```bash
Delta-wing mixer for PX4FMU
===========================

Designed for Wing Wing Z-84

This file defines mixers suitable for controlling a delta wing aircraft using
PX4FMU. The configuration assumes the elevon servos are connected to PX4FMU
servo outputs 0 and 1 and the motor speed control to output 3. Output 2 is
assumed to be unused.

Inputs to the mixer come from channel group 0 (vehicle attitude), channels 0
(roll), 1 (pitch) and 3 (thrust).

See the README for more information on the scaler format.

Elevon mixers
-------------
Three scalers total (output, roll, pitch).

On the assumption that the two elevon servos are physically reversed, the pitch
input is inverted between the two servos.

The scaling factor for roll inputs is adjusted to implement differential travel
for the elevons.

M: 2
O:      10000  10000      0 -10000  10000
S: 0 0  -6000  -6000      0 -10000  10000
S: 0 1   6500   6500      0 -10000  10000

M: 2
O:      10000  10000      0 -10000  10000
S: 0 0  -6000  -6000      0 -10000  10000
S: 0 1  -6500  -6500      0 -10000  10000

Output 2
--------
This mixer is empty.

Z:

Motor speed mixer
-----------------
Two scalers total (output, thrust).

This mixer generates a full-range output (-1 to 1) from an input in the (0 - 1)
range.  Inputs below zero are treated as zero.

M: 1
O:      10000  10000      0 -10000  10000
S: 0 3      0  20000 -10000 -10000  10000

```
