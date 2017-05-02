---
translated_page: https://github.com/PX4/Devguide/blob/master/en/uavcan/README.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# UAVCAN Introduction

![](../../assets/uavcan-logo-transparent.png)

[UAVCAN](http://uavcan.org) 是一种板载网络，允许自驾仪通过该协议连接各类航空电子仪器，其支持的硬件有：

- 电机控制器
  - [Pixhawk ESC](https://pixhawk.org/modules/pixhawk_esc)
  - [SV2740 ESC](https://github.com/thiemar/vectorcontrol)
- 空速管传感器
  - [Thiemar 空速管](https://github.com/thiemar/airspeed)
- GPS和GLONASS的GNSS接收器
  - [Zubax GNSS](http://zubax.com/product/zubax-gnss)

相较于爱好级设备而言，UAVCAN使用坚固的差分信号，并通过总线支持固件升级。所有电机控制器ESC均能提供反馈（数值式信号并直接控制电机转速）和转子磁通定向控制FOC应用（FOC专为高性能电机应用而开发，实现高精度矢量控制）。


## 初始设置


以下说明提供连接和设置四轴飞行器的分步指南，其中ESC和GPS通过UAVCAN连接。硬件选取：Pixhawk 2.1、Zubax Orel 20 ESCs和Zubax GNSS GPS模块。


### 连线

第一步是将所有启用UAVCAN的设备与飞行控制器连接。下图显示如何连接所有组件，所使用的Zubax器件都支持冗余CAN接口，其中第二总线是可选的，但增加了连接的鲁棒性。

![](../../assets/UAVCAN_wiring.png)

值得注意的是，一些设备需要外部电源供电（例如Zubax Orel 20电调需连接动力电池），而其他设备可由CAN供电（例如Zubax GNSS）。请在继续设置前，参考相关硬件文档。


### 固件设置

然后，按照[UAVCAN配置](../uavcan/node_enumeration.md)中的说明激活固件中的UAVCAN功能。断开电源并重新连接，在重新上电后，所有UAVCAN设备将被检查到，可以通过连接Orel 20 电调上的电机蜂鸣确认。现在可以继续进行常规设置和校准。

根据所使用的硬件，在UAVCAN设备上执行固件更新是合理的。这可以通过UAVCAN本身和PX4固件来完成。有关详细信息，请参阅[UAVCAN固件](../uavcan/node_firmware.md)中的说明。

## 升级节点固件

当提供匹配的固件时，PX4中间件将自动升级UAVCAN节点上的固件。过程和要求在[UAVCAN 固件](../uavcan/node_firmware.md)页面中有相关描述。


## 编号并配置电机控制器

在简单的设置操作完成后，可以多每个电机控制器ID和旋转方向进行分配：[UAVCAN节点编号](../uavcan/node_enumeration.md)。该操作可以由用户通过QGroundControl启动。

## 有用的链接

* [Homepage](http://uavcan.org)
* [Specification](http://uavcan.org/Specification)
* [Implementations and tutorials](http://uavcan.org/Implementations)


