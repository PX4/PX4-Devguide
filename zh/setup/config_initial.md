# 初始配置

我们建议开发者们获取下文描述的基本配置的硬件设备（或者相似的设备）并使用"默认" [机架](../airframes/airframe_reference.md) 构型。 

## 基本设备

> **提示：** 除了这里提及的设备外 PX4 还适用于很多其他硬件设备，但新晋开发人员可以受益于使用下文的标准配置进行开发。 一个Taranis RC 遥控器加上一个 Note 4 平板电脑可以组成一套物美价廉的外场套件。

强烈建议使用以下硬件设备:

* 一个供安全飞行员（或等效职能人员）使用的 Taranis Plus Rm 遥控器
* 开发用计算机： 
  * MacBook Pro （2015 年初及以后），OSX 10.13 或者更高版本
  * Lenovo Thinkpad 450 (i5)，Ubuntu Linux 16.04 或者更高版本
* 一个地面控制站设备: 
  * iPad （需要 Wifi 无线适配器）
  * Any MacBook or Ubuntu Linux laptop (can be the development computer)
  * Samsung Note 4 or equivalent (any recent Android tablet or phone with a large enough screen to run *QGroundControl* effectively).
* Safety glasses
* For multicopters - tether for more risky tests

## Vehicle Configuration

> **Tip** *QGroundControl* for a **desktop OS** is required for vehicle configuration. You should use (and regularly update) the daily build in order to take advantage of the latest features in PX4.

To configure the vehicle:

1. Download the [QGroundControl Daily Build](https://docs.qgroundcontrol.com/en/releases/daily_builds.html) for your development platform.
2. [Basic Configuration](https://docs.px4.io/en/config/) (PX4 User Guide) explains how to to perform basic configuration. 
3. [Parameter Configuration](https://docs.px4.io/en/advanced_config/parameters.html) (PX4 User Guide) explains how you can find and modify individual parameters.