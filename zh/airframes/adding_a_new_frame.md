---
translated_page: https://github.com/PX4/Devguide/blob/master/en/airframes/adding_a_new_frame.md
translated_sha: f7d0be49d427db1a07e35167f8fe7e861d577b27
---

# 添加一个新的机型


PX4使用存储的配置作为机型的起始点。添加配置是非常简单的：在[init.d文件夹](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d)创建一个新的文件，这个文件需要以一个没有使用的自动启动ID作为文件名的前缀，然后[构建并上传](../setup/building_px4.md)固件即可。

如果不想创建自己的配置文件，也可以用SD卡上的文本文件替换掉已有的自定义配置文件，具体细节请查看[自定义系统启动](../advanced/system_startup.md)页。

## 机型配置

一个机型配置包括3项基本内容：

- 应该启动的应用，例如多旋翼或者固定翼的控制器
- 系统（固定翼，飞翼或者多旋翼）的物理配置，这叫做混控器
- 参数整定

这三方面大多数时候是独立的，也就是说，许多配置会共享相同的机型物理布局以及启动相同的应用，它们之间最大的不同在参数整定部分。

所有的配置存储在[ROMFS/px4fmu_common/init.d](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d)文件夹。所有的混控器存储在[ROMFS/px4fmu_common/mixers](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/mixers)文件夹。

### 配置文件

如下所示，是一个典型的配置文件：

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

# Configure this as plane
set MAV_TYPE 1
# Set mixer
set MIXER wingwing
# Provide ESC a constant 1000 us pulse
set PWM_OUT 4
set PWM_DISARMED 1000
```
> **Warning** 如果要反转通道，请勿在RC遥控器发射器或者RC1_REV上进行。通道只会在以手动模式飞行时反转，当你切换到一个自驾仪飞行模式时，通道输出仍然会出错(只会反转遥控器信号)。因此，为了正确地分配通道，可以使用PWM_MAIN_REV1(例如通道1)改变PWM的值，或者更改相应混控器中输出缩放值和输出 范围的符号(如下图)。

### 混控器文件

一个典型的混控器文件会像下面这样：

> **注意：** 舵机/电机的接口顺序和这个文件中的混控器顺序一致。


所以MAIN1对应左副翼，MAIN2对应右副翼，MAIN3置空（注意：Z即为空混控器），MAIN4则对应油门（对于一般固定翼配置，保持油门和输出4对应）。

混控器被编码为从-10000到10000的标准单位，对应-1到+1。

```
M: 2
O:      10000  10000      0 -10000  10000
S: 0 0  -6000  -6000      0 -10000  10000
S: 0 1   6500   6500      0 -10000  10000
```

从左到右每个数字代表的意思如下：

- M：代表有2个缩放系数（对应着两个输入）
- O：代表输出缩放系数（负输入量缩放系数为1，正输入量缩放系数为1），偏移量（这里是0），输出范围（这里-1到+1）
- S：代表第一个输入量的缩放系数：输入量来自控制组#0（姿态控制）的第一个输入（滚转），缩放系数为0.6，并且符号取反（-0.6换算到标准单位是-6000），没有偏移量（0），输出为全范围（-1到+1）
- S：代表第二个输入量的缩放系数：输入量来自控制组#0（姿态控制）的第二个输入（俯仰），缩放系数为0.65（0.65换算到标准单位是6500），没有偏移量（0），输出为全范围（-1到+1）

所有的缩放器结果累加，对飞翼而言，控制面偏移量取滚转信号的60%和俯仰信号的65%。如果俯仰信号和滚转信号都取最大值，那么偏移量将达到125%，超出了输出范围，这就意味着第一个通道（滚转）比第二个通道（俯仰）优先级高。

完整的混控器定义如下：

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

### 让新的机型在QGroundControl中显示

机型的元数据捆绑在.px4固件文件中（这是一个压缩的JSON文件）。

> **注意：** 确保在QGroundControl（自定义文件选项）中刷写生成的.px4文件以将元数据加载到应用程序中。 然后，新的机型将在用户界面中可用。