# 各种说明

这是在设置或使用 UAVCAN 时解决问题的一系列提示和技巧。

### 解锁，但电机不旋转

如果 px4 固件解锁，但电机无法开始旋转，请检查参数 **UAVCAN\_ENABLE**。 它应该设置为 3，以便使用通过 UAVCAN 连接的电调作为输出。 此外，如果电机在增加推力之前没有开始旋转，请检查 **UAVCAN\_ESC\_IDLT** 并将其设置为 1。

### 用 Zubax Babel 进行调试

A great tool to debug the transmission on the UAVCAN bus is the [Zubax Babel](https://zubax.com/products/babel) in combination with the [GUI tool](http://uavcan.org/GUI_Tool/Overview/). 它们还可以独立于 Pixhawk 硬件使用，以测试节点或手动控制启用了 UAVCAN 的电调。

### UAVCAN devices dont get node ID/FW doesn't update

PX4 requires an SD card for UAVCAN node allocation and firmware upgrade (both of which happen during boot). Check that there is a (working) SD card present and reboot.