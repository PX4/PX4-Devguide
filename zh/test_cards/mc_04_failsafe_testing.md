# 测试 MC_04 -故障安全测试

❏ 验证 RC Loss 动作是否返回陆地

❏ 验证数据链路丢失操作返回陆地，超时是10秒

❏ 验证电池是否安全

&nbsp;&nbsp;&nbsp;&nbsp;❏ 行动是返回土地

&nbsp;&nbsp;&nbsp;&nbsp;❏ 电池警告等级为 25％

&nbsp;&nbsp;&nbsp;&nbsp;❏ 电池故障安全等级为 20％

&nbsp;&nbsp;&nbsp;&nbsp;❏ 电池紧急等级为 15％

❏ 在高度模式下起飞

❏ 离开原位至少 20 米

❏ RC 损失

&nbsp;&nbsp;&nbsp;&nbsp;❏ 关闭 RC 并检查设备返回原位，等待下降并打开 RC 并接管。

## 数据链路丢失

❏ 断开遥测，车辆应在 10 秒后返回原位，等待下降并重新连接遥测无线电

## 切换到海拔高度模式

❏ 确保侧倾，俯仰和偏航杆像稳定模式一样响应

❏ Throttle should control altitude, and when the stick is centered it must maintain altitude

## Switch to Position Mode

❏ When the sticks are centered, it must maintain position

❏ Move roll, pitch and yaw and check the vehicle is moving according to the inputs

❏ Center the sticks again and check the vehicle maintains position

## Wait for Battery Failsafe to Trigger

❏ Confirm the warning message is received in QGC

❏ Confirm the vehicle returns to land

❏ Confirm the vehicle lands.