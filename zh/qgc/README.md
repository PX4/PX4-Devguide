# QGroundControl

QGroundControl 是一个应用程序来配置和飞行基于 PX4 的自动驾驶仪。 跨平台支持所有主流操作系统:

- 手机：安卓和 iOS（目前专注于平板电脑）
- 桌面：Windows，Linux，Mac OS

## 规划任务

要规划新任务，请切换到规划选项卡，单击左上角的 + 图标，然后单击地图创建航点。 在侧面打开一个上下文菜单来调整航点。 点击高亮传输图标，将任务发送到飞机上。

![](../../assets/gcs/planning-mission.png)

## 执行飞行任务

切换到最右侧的标签页。 地图上应该可以看到任务。 点击更改当前飞行模式下发到任务中，并且解锁飞机。 如果飞机已经起飞，它会飞到任务的第一段再执行。

![](../../assets/gcs/flying-mission.png)

## 设置参数

切换到设置标签。 将左侧菜单拉到最下，点击参数图标。 参数可以通过双击来更改，这会打开一个带有详细描述的可编辑的文本菜单。

![](../../assets/gcs/setting-parameter.png)

## 安装

QGroundControl 可以从 [网站](http://qgroundcontrol.com/downloads) 下载。

> **Tip** 推荐开发者使用最新编译版本替代稳定版。

## 由源代码安装

为了匹配最近版本的飞控代码，鼓励固件开发者从源码编译。

Follow the [QGroundControl build instructions](https://dev.qgroundcontrol.com/en/getting_started/) to install Qt and build the source code.