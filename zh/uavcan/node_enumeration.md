---
translated_page: https://github.com/PX4/Devguide/blob/master/en/uavcan/node_enumeration.md
translated_sha: 9a3793a4e3fc7b666e741aa333578c7f08b9e875
translated: true
---

# UAVCAN枚举与配置

> **提示：** 如下图所示通过勾选`Enable UAVCAN`复选框，将UAVCAN作为默认电机输出总线。或者可以在*QGroundControl*参数编辑器中将`UAVCAN_ENABLE`参数设置为`3`。 若将其设置为`2`虽然能启用CAN，但电机输出仍为PWM。

使用 [QGroundControl](../qgc/README.md)并切换到设置界面，选择左侧的电源配置，点击`开始分配(start assignment)`按钮。

第一次发出哔声后，将第一个ESC上的电机桨叶快速朝正确方向转动。 每次编号完成后，ESC都将发出蜂鸣声。对所有电机控制器按[电机映射图](../airframes/airframe_reference.md)上显示的顺序重复此步骤。此步骤只能执行一次，不需要在固件升级后重复。



![UAVCAN Enumeration Controls (bottom right of image)](../../assets/uavcan-qgc-setup.png)
