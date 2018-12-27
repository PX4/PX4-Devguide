# 测试 MC_03 - 自动手动混合

## 创建和上传任务

❏ 任务标准

&nbsp;&nbsp;&nbsp;&nbsp;❏ 整个任务期间高度的变化

&nbsp;&nbsp;&nbsp;&nbsp;❏ 任务应该在空中结束，而不是Land/RTL

&nbsp;&nbsp;&nbsp;&nbsp;❏ 持续时间为 3 到 4 分钟

❏ 使用* QGroundControl *将任务上传到无人机

## 飞行

❏ 位置模式下的摆臂和起飞

❏ 参与自动模式

❏ Observe tracking and cornering

❏ Once mission has completed, switch back to Position mode

&nbsp;&nbsp;&nbsp;&nbsp;❏ Horizontal position should hold current value with stick centered

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vertical position should hold current value with stick centered

&nbsp;&nbsp;&nbsp;&nbsp;❏ Throttle response set to Climbs/Descend rate

&nbsp;&nbsp;&nbsp;&nbsp;❏ Pitch/Roll/Yaw response set to Pitch/Roll/Yaw rates

❏ Engage RTL

❏ Upon touching ground, copter should disarm automatically within 2 seconds (disarm time set by parameter: [COM_DISARM_LAND](../advanced/parameter_reference.md#COM_DISARM_LAND))

## Expected Results

* Take-off should be smooth as throttle is raised
* No oscillations should present in any of the above flight modes
* Upon landing, copter should not bounce on the ground