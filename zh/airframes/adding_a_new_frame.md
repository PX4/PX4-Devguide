# 添加一个新的机型

PX4使用存储的配置作为机型的起始点 机体的配置在[ROMFS/px4fmu_common/init.d](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d)文件夹下的[配置文件](#config-file)中定义。 配置文件中引用[混控文件](#mixer-file)，混控文件是用来描述机体的物理结构，存储在[ROMFS/px4fmu_common/mixers](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/mixers)文件夹

添加配置是非常简单的：在[init.d文件夹](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d)创建一个新的文件(使用未使用的autostart ID作为文件名的前缀)，然后[构建并上传](../setup/building_px4.md)固件即可。

如果不想创建自己的配置文件，也可以用SD卡上的文本文件替换掉已有的自定义配置文件，具体细节请查看[自定义系统启动页。](../concept/system_startup.md)

## 配置文件概述

配置和混控文件中的机型配置包括包括如下几个主要模块：

* 机架说明文档(被[Airframes Reference](../airframes/airframe_reference.md)和*QGroundControl*) 使用。
* 飞机特定的参数设置，包括[tuning gains](#tuning-gains)。
* 应该启动的应用，例如多旋翼或者固定翼的控制器，着陆检测等等。
* 系统（固定翼，飞翼或者多旋翼）的物理配置。 这叫[混控器](../concept/mixing.md)。

These aspects are mostly independent, which means that many configurations share the same physical layout of the airframe, start the same applications and differ most in their tuning gains.

> **Note** New airframe files are only automatically added to the build system after a clean build (run `make clean`).

### Config File {#config-file}

A typical configuration file is shown below ([original file here](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d/3033_wingwing)) .

The first section is the airframe documentation. This is used in the [Airframes Reference](../airframes/airframe_reference.md) and *QGroundControl*.

```bash
#!nsh
#
# @name Wing Wing (aka Z-84) Flying Wing
#
# @url https://docs.px4.io/en/framebuild_plane/wing_wing_z84.html
#
# @type Flying Wing
# @class Plane
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
```

The next section specifies vehicle-specific parameters, including [tuning gains](#tuning-gains):

```bash
sh /etc/init.d/rc.fw_defaults

if [ $AUTOCNF == yes ]
then
  param set BAT_N_CELLS 2
  param set FW_AIRSPD_MAX 15
  param set FW_AIRSPD_MIN 10
  param set FW_AIRSPD_TRIM 13
  param set FW_R_TC 0.3
  param set FW_P_TC 0.3
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
```

Set frame type ([MAV_TYPE](https://mavlink.io/en/messages/common.html#MAV_TYPE)):

```bash
# Configure this as plane
set MAV_TYPE 1
```

Set the [mixer](#mixer-file) to use:

```bash
# Set mixer
set MIXER wingwing
```

Configure PWM outputs (specify the outputs to drive/activate, and the levels).

```bash
# Provide ESC a constant 1000 us pulse
set PWM_OUT 4
set PWM_DISARMED 1000
```

> **Warning** If you want to reverse a channel, never do this on your RC transmitter or with e.g `RC1_REV`. The channels are only reversed when flying in manual mode, when you switch in an autopilot flight mode, the channels output will still be wrong (it only inverts your RC signal). Thus for a correct channel assignment change either your PWM signals with `PWM_MAIN_REV1` (e.g. for channel one) or change the signs of the output scaling in the corresponding mixer (see below).

### Mixer File {#mixer-file}

> **Note** First read [Concepts > Mixing](../concept/mixing.md). This provides background information required to interpret this mixer file.

A typical mixer file is shown below ([original file here](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/mixers/wingwing.main.mix)). A mixer filename, in this case `wingwing.main.mix`, gives important information about the type of airframe (`wingwing`), the type of output (`.main` or `.aux`) and lastly that it is a mixer file (`.mix`).

The mixer file contains several blocks of code, each of which refers to one actuator or ESC. So if you have e.g. two servos and one ESC, the mixer file will contain three blocks of code.

> **Note** The plugs of the servos / motors go in the order of the mixers in this file.

So MAIN1 would be the left aileron, MAIN2 the right aileron, MAIN3 is empty (note the Z: zero mixer) and MAIN4 is throttle (to keep throttle on output 4 for common fixed wing configurations).

A mixer is encoded in normalized units from -10000 to 10000, corresponding to -1..+1.

    M: 2
    O:      10000  10000      0 -10000  10000
    S: 0 0  -6000  -6000      0 -10000  10000
    S: 0 1   6500   6500      0 -10000  10000
    

Where each number from left to right means:

* M: Indicates two scalers for two control inputs. It indicates the number of control inputs the mixer will receive.
* O: Indicates the output scaling (*1 in negative, *1 in positive), offset (zero here), and output range (-1..+1 here).  
  * If you want to invert your PWM signal, the signs of the output scalings have to be changed. (```O:      -10000  -10000      0 -10000  10000```)
  * This line can (and should) be omitted completely if it specifies the default scaling: ```O:      10000  10000   0 -10000  10000```
* S: Indicates the first input scaler: It takes input from control group #0 (Flight Control) and the first input (roll). It scales the roll control input * 0.6 and reverts the sign (-0.6 becomes -6000 in scaled units). It applies no offset (0) and outputs to the full range (-1..+1)
* S: Indicates the second input scaler: It takes input from control group #0 (Flight Control) and the second input (pitch). It scales the pitch control input * 0.65. It applies no offset (0) and outputs to the full range (-1..+1)

> **Note** In short, the output of this mixer would be SERVO = ( (roll input * -0.6 + 0) + (pitch input * 0.65 + 0) ) * 1 + 0

Behind the scenes, both scalers are added, which for a flying wing means the control surface takes maximum 60% deflection from roll and 65% deflection from pitch.

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

The scaling factor for roll inputs is adjusted to implement differential travel
for the elevons. 

This first block of code is for Servo 0...

M: 2
O:      10000  10000      0 -10000  10000
S: 0 0  -6000  -6000      0 -10000  10000
S: 0 1   6500   6500      0 -10000  10000

And this is for Servo 1...

M: 2
O:      10000  10000      0 -10000  10000
S: 0 0  -6000  -6000      0 -10000  10000
S: 0 1  -6500  -6500      0 -10000  10000

Note that in principle, you could implement left/right wing asymmetric mixing, but in general the two blocks of code will be numerically equal, and just differ by the sign of the third line (S: 0 1), since to roll the plane, the two ailerons must move in OPPOSITE directions. The signs of the second lines (S: 0 0) are indentical, since to pitch the plane, both servos need to move in the SAME direction. 

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

## Tuning Gains

The following *PX4 User Guide* topics explain how to tune the parameters that will be specified in the config file:

* [Multicopter PID Tuning Guide](https://docs.px4.io/en/advanced_config/pid_tuning_guide_multicopter.html)
* [Fixed Wing PID Tuning Guide](https://docs.px4.io/en/advanced_config/pid_tuning_guide_fixedwing.html)
* [VTOL Configuration](https://docs.px4.io/en/config_vtol/)

## Add New Airframe to QGroundControl

To make a new airframe available for section in the *QGroundControl* [airframe configuration](https://docs.px4.io/en/config/airframe.html):

1. Make a clean build (e.g. by running `make clean` and then `make px4fmu-v5_default`)
2. Open QGC and select **Custom firmware file...** as shown below:
  
  ![QGC flash custom firmware](../../assets/gcs/qgc_flash_custom_firmware.png)
  
  You will be asked to choose the **.px4** firmware file to flash (this file is a zipped JSON file and contains the airframe metadata).

3. Navigate to the build folder and select the firmware file (e.g. **Firmware/build/nuttx_px4fmu-v5_default/px4fmu-v5_default.px4**).

4. Press **OK** to start flashing the firmware.
5. Restart *QGroundControl*.

The new airframe will then be available for selection in *QGroundControl*.