# PX4 Platform Hardware/Software Architecture

The diagram below provides a forward-looking high level overview of a full PX4 onboard and offboard stack. 图的左侧展示的是一种可能的 *飞行控制器 （flight controller）* （亮蓝色）通过 [RTPS](../middleware/micrortps.md) 与 *视觉感知计算机（perception computer）* (深蓝色) 相连接的硬件配置。 感知计算机配备一个单独的相机载荷，并使用相机传感器阵列提供视觉控制和目标回避功能。

图的右侧显示了端到端的软件堆栈。 The stack "approximately" aligns horizontally with the hardware parts of the diagram, and is colour-coded to show which software is running on the flight controller and which on the companion computer.

> **Note** [PX4 Architectural Overview](../concept/architecture.md) 页面提供了有关飞行控制栈和中间件的相关信息。 Offboard APIs are covered in [ROS](../ros/README.md) and [MAVSDK](https://mavsdk.mavlink.io/develop/en/index.html).

![PX4 Platform architecture](../../assets/diagrams/dronecode_platform_architecture.jpg)

<!-- The drawing is on draw.io: https://drive.google.com/file/d/14sgSpcs7NcBatW-qn0dLtyMHvwNMSSlm/view?usp=sharing. Request access from dev team. -->