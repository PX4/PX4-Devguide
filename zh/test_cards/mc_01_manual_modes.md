# 测试 MC_01 - 手动模式

## 解锁并起飞

❏ 设置飞行模式以稳定和布防

❏ 升高油门起飞

## 飞行

❏ 自稳

&nbsp;&nbsp;&nbsp;&nbsp;❏ 俯仰/滚转/偏航响应 1：1

&nbsp;&nbsp;&nbsp;&nbsp;❏ 节气门响应 1：1

❏ Altitude

&nbsp;&nbsp;&nbsp;&nbsp;❏ 垂直位置应以棒为中心保持当前值

&nbsp;&nbsp;&nbsp;&nbsp;❏ Pitch/Roll/Yaw response 1:1

&nbsp;&nbsp;&nbsp;&nbsp;❏ Throttle response set to Climbs/Descend rate

❏ 定点

&nbsp;&nbsp;&nbsp;&nbsp;❏ Horizontal position should hold current value with stick centered

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vertical position should hold current value with stick centered

&nbsp;&nbsp;&nbsp;&nbsp;❏ Throttle response set to Climbs/Descend rate

&nbsp;&nbsp;&nbsp;&nbsp;❏ Pitch/Roll/Yaw response set to Pitch/Roll/Yaw rates

## Landing

❏ Land in Position mode with the throttle below 40%

❏ Upon touching ground, copter should disarm automatically within 2 seconds (disarm time set by parameter: [COM_DISARM_LAND](../advanced/parameter_reference.md#COM_DISARM_LAND))

## Expected Results

* Take-off should be smooth as throttle is raised
* No oscillations should present in any of the above flight modes
* Upon landing, copter should not bounce on the ground