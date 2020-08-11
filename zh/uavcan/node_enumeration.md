# UAVCAN 列举与配置

> **Note** 通过勾选“启用UAVCAN”复选框启用UAVCAN作为默认电机输出总线，如下所示。 可以在 *QGroundControl* 参数编辑器中将 UAVCAN_ENABLE 参数设置为“3”。 将其设置为“2”以启用 CAN 总线，但将电机输出保持为 PWM。

使用 [QGroundControl](../qgc/README.md) 并切换到“设置”视图。 选择左侧的电源配置。 单击“开始分配”按钮。

第一次发出蜂鸣声后，将第一个电调上的螺旋桨迅速转到正确的转向。 每次列举时，电调都会发出蜂鸣声。 按照 [motor map](../airframes/airframe_reference.md) 所示的顺序对所有电机控制器重复此步骤。 运行 Sapog 固件的电调需要在列举后重新启动，以便应用新的列举ID。 此步骤只需执行一次，固件升级后无需重复。

![UAVCAN列举控件（图像右下角）](../../assets/uavcan-qgc-setup.png)