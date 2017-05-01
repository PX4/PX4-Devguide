# 各种笔记


这是一个提示和技巧的集合，以解决在设置或使用UAVCAN时的问题。

### 启动但电机不旋转

如果PX4固件启动，但电机没有开始旋转，请检查参数**UAVCAN_ENABLE**（应将其设置为`3`，以便使用通过UAVCAN连接的ESC作为输出）。此外，如果电机在推力增加之前不开始旋转，请检查参数**UAVCAN_ESC_IDLT，**并将其设置为`1`。

### 使用Zubax Babel进行调试


调试UAVCAN总线上的数据传输的一个好方法是[Zubax Babel](https://docs.zubax.com/zubax_babel)与[GUI工具](http://uavcan.org/GUI_Tool/Overview/)的组合使用。它们也可以独立于Pixhawk硬件使用，以便测试节点或手动控制启用了UAVCAN的ESC。


































