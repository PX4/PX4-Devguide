---
translated_page: https://github.com/PX4/Devguide/blob/master/en/airframes/architecture.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# 机型概述


PX4系统是模块化的架构，这使得它对所有的机器人类型都可以使用同一个代码库。

{% mermaid %}
graph LR;
  Autopilot-->Controller;
  SafetyPilot-->Controller;
  Controller-->Mixer;
  Mixer-->Actuator
{% endmermaid %}

## 基本设备

在机型部分所用到的硬件包括以下基本设备：

- 1个Taranis Plus遥控器（或者其它有PPM/S.BUS输出的设备），用于保证安全飞行。
- 1个地面站
  - Samsung Note 4或者同类型的较新的Android平板
  - iPad（需要无线遥测适配器）
  - 任何MacBook或者Ubuntu Linux笔记本
- 1台in-field电脑（用于软件开发者）
  - MacBook Pro或者Air，至少OS X 10.10
  - 现代Ubuntu Linux笔记本，至少14.04
- 安全眼镜
  - 用于多旋翼危险测试

PX4应用范围很广，但是对于新手开发者而言，从标准配置做起会更好，1个Taranis Plus遥控器，1个Note 4平板就可以组成一套便宜的套件。