# PX4 开发指南

[![版本发布](https://img.shields.io/github/release/PX4/Firmware.svg)](https://github.com/PX4/Firmware/releases) [![讨论](https://img.shields.io/badge/discuss-px4-ff69b4.svg)](http://discuss.px4.io/) [![Slack](https://px4-slack.herokuapp.com/badge.svg)](http://slack.px4.io)

> **Info**: 本手册主要面向软件开发人员及(新) 硬件集成商。 想要使用支持飞行、编译和更改的飞行器，请参阅[PX4用户手册](https://docs.px4.io/en/)

本指南介绍了如下内容：

* 了解到[配置最小开发环境](setup/config_initial.md), [用源码编译PX4](setup/building_px4.md) 以及部署[众多支持的自动驾驶仪](https://docs.px4.io/en/flight_controller/).
* 理解[PX4系统架构](concept/architecture.md)以及核心概念。
* 学习如何更改飞行栈及中间层： 
  * 更改飞行算法和添加新的[飞行模式](concept/flight_modes.md)。
  * 支持新的[飞行器](airframes/README.md)。
* 学到如何在新的硬件上集成PX4： 
  * 支持新的传感器和执行器, 包括摄像头、测距仪等。
  * 修改PX4使之能够在新的自驾仪硬件上运行。
* 对PX4进行[ 仿真 ](simulation/README.md)、[ 测试 ](test_and_ci/README.md) 和 [ 调试/查看日志 ](debug/README.md)。
* 与外部机器人的api 进行联调通信/集成。

## 论坛和聊天 {#support}

核心开发团队和社区活跃与以下论坛和聊天频道。

* [PX4 Discuss](http://discuss.px4.io/)（*推荐*）
* [Slack](http://slack.px4.io)（注册链接）
* [Google+](https://plus.google.com/117509651030855307398)

> ** 提示 **希望为平台做 [ 贡献 ](contribute/README.md)的开发人员来参加 [ 周开发交流 ](contribute/README.md#dev_call) 和我们的其他 [ 开发人员活动 ](contribute/README.md#calendar) 也是最受欢迎的 。

## 贡献

[ 贡献 & 开发人员交流 ](contribute/README.md) 解释了如何用我们的源代码工作。 [ 文档 ](contribute/docs.md) 解释了文档更改的方法和位置。

## 翻译

本指南中有中文和韩文 [ 翻译 ](contribute/docs.md#translation)。 您可以通过单击语言切换器图标来访问这些内容:

![Gitbook 语言选择器](../assets/gitbook/gitbook_language_selector.png)

## 授权条款

在[BSD 3-clause license](https://opensource.org/licenses/BSD-3-Clause)的条款许可下，该代码可以自由使用和修改。 文档已获得[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)许可。 更多信息请参见: [ 许可证 ](contribute/licenses.md)。

## 管理

PX4飞行栈受[Dronecode项目](https://www.dronecode.org/)管理。

<a href="https://www.dronecode.org/" style="padding:20px"><img src="https://mavlink.io/assets/site/logo_dronecode.png" alt="Dronecode Logo" width="110px"/></a>
<a href="https://www.linuxfoundation.org/projects" style="padding:20px;"><img src="https://mavlink.io/assets/site/logo_linux_foundation.png" alt="Linux Foundation Logo" width="80px" /></a>

<div style="padding:10px">&nbsp;</div>
