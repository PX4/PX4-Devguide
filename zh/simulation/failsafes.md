# 模拟故障保护

[Failsafes](https://docs.px4.io/master/en/config/safety.html) define the safe limits/conditions under which you can safely use PX4, and the action that will be performed if a failsafe is triggered (for example, landing, holding position, or returning to a specified point).

在 SITL 中，默认情况下会禁用某一些故障，以便方便模拟使用。 本主题说明如何在实际世界中尝试 SITL 仿真之前测试安全关键行为。

> **Note** 您还可以使用 [ HITL 模拟](../simulation/hitl.md) 测试故障。 HITL 使用飞行控制器的常规配置参数。

<span></span>

> **Tip** The [SITL parameters](../advanced/parameter_reference.md#sitl) allow you to simulate other common sensor failure cases that are not covered here, including: loss of barometer, gyro and accelerometer, increased GPS noise etc.

## 数据链路丢失

The *Data Link Loss* failsafe (unavailability of external data via MAVLink) is enabled by default. This makes the simulation only usable with a connected GCS, SDK, or other MAVLink application.

Set the parameter [NAV_DLL_ACT](../advanced/parameter_reference.md#NAV_DLL_ACT) to the desired failsafe action to change the behavior. For example, set to `0` to disable it.

> **Note** 当您执行 `make clean` 时，SITL 中的所有参数（包括此参数）都会被重置。

## RC 链接损失

The *RC Link Loss* failsafe (unavailability of data from a remote control) is enabled by default. This makes the simulation only usable with either an active MAVLink or remote control connection.

Set the parameter [NAV_RCL_ACT](../advanced/parameter_reference.md#NAV_RCL_ACT) to the desired failsafe action to change the behavior. For example, set to `0` to disable it.

> **Note** All parameters in SITL including this one get reset when you do `make clean`.

## 低电量

The simulated battery is implemented to never run out of energy, and by default only depletes to 50% of its capacity and hence reported voltage. This enables testing of battery indication in GCS UIs without triggering low battery reactions that might interrupt other testing.

To change this minimal battery percentage value change [this line](https://github.com/PX4/Firmware/blob/9d67bbc328553bbd0891ffb8e73b8112bca33fcc/src/modules/simulator/simulator_mavlink.cpp#L330).

To control how fast the battery depletes to the minimal value use the parameter [SIM_BAT_DRAIN](../advanced/parameter_reference.md#SIM_BAT_DRAIN).

> **Tip** By changing this configuration in flight, you can also test regaining capacity to simulate inaccurate battery state estimation or in-air charging technology.

## GPS 损失

To simulate losing and regaining GPS information you can just stop the publication of GPS messages. This is done by running the `param set SIM_GPS_BLOCK 1` and `param set SIM_GPS_BLOCK 0` commands on your SITL instance *pxh shell* to block and unblock messages respectively.