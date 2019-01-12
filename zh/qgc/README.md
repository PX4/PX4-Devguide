# QGroundControl

QGroundControl 是一个应用程序来配置和飞行基于 PX4 的自动驾驶仪。 跨平台支持所有主流操作系统:

- 手机: 安卓和 iOS (目前专注于平板电脑)
- 桌面: windows, linux, mac os

## 规划任务

要规划新任务，请切换到规划选项卡，单击左上角的 + 图标，然后单击地图创建航点。 在侧面打开一个上下文菜单来调整航点。 点击高亮传输图标，将任务发送到飞机上。

![](../../assets/gcs/planning-mission.png)

## 执行飞行任务

切换到最右侧的标签页。 地图上应该可以看到任务。 点击更改当前飞行模式下发到任务中，并且解锁飞机。 如果飞机已经起飞，它会飞到任务的第一段再执行。

![](../../assets/gcs/flying-mission.png)

## 设置参数

切换到设置标签。 将左侧菜单拉到最下，点击参数图标。 Parameters can be changed by double-clicking on them, which opens a context menu to edit, along with a more detailed description.

![](../../assets/gcs/setting-parameter.png)

## 安装

QGroundControl can be downloaded from its [website](http://qgroundcontrol.com/downloads).

> **Tip** Developers are advised to use the latest daily build instead of the stable release.

## 由源代码安装

Firmware developers are encouraged to build from source in order to have a matching recent version to their flight code.

Follow the [QGroundControl build instructions](https://github.com/mavlink/qgroundcontrol#obtaining-source-code) to install Qt and build the source code.