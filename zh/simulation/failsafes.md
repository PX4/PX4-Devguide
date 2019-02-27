# 模拟故障保护

[ Failsafes ](https://docs.px4.io/en/config/safety.html) 为了您可以安全地使用 PX4，定义安全限制/条件，以及触发故障安全时将执行的操作（例如，着陆，保持或返回指定点）。

在 SITL 中，默认情况下会禁用某一些故障，以便方便模拟使用。 本主题说明如何在实际世界中尝试 SITL 仿真之前测试安全关键行为。

> **Note** 您还可以使用 [ HITL 模拟](../simulation/hitl.md) 测试故障。 HITL 使用飞行控制器的常规配置参数。

## 数据链路丢失

默认情况下启用 *数据链路丢失* 故障保护（无法通过 MAVLink 获取外部数据）。 这使得模拟仅适用于连接的 GCS，SDK 或其他 MAVLink 应用程序。

将参数 [NAV_DLL_ACT](../advanced/parameter_reference.md#NAV_DLL_ACT) 设置为想要的故障保护操作，以改变行为。 例如，设置为 `0` 禁用它。

> **Note** All parameters in SITL including this one get reset when you do `make clean`.

## RC Link Loss

The *RC Link Loss* failsafe (unavailability of data from a remote control) is enabled by default. This makes the simulation only usable with either an active MAVLink or remote control connection.

Set the parameter [NAV_RCL_ACT](../advanced/parameter_reference.md#NAV_RCL_ACT) to the desired failsafe action to change the behavior. For example, set to `0` to disable it.

> **Note** All parameters in SITL including this one get reset when you do `make clean`.

## Low Battery

The simulated battery is implemented to never run out of energy, and by default only depletes to 50% of its capacity and hence reported voltage. This enables testing of battery indication in GCS UIs without triggering low battery reactions that might interrupt other testing.

To change this minimal battery percentage value change [this line](https://github.com/PX4/Firmware/blob/9d67bbc328553bbd0891ffb8e73b8112bca33fcc/src/modules/simulator/simulator_mavlink.cpp#L330).

To control how fast the battery depletes to the minimal value use the parameter [SIM_BAT_DRAIN](../advanced/parameter_reference.md#SIM_BAT_DRAIN).

> **Tip** By changing this configuration in flight, you can also test regaining capacity to simulate inaccurate battery state estimation or in-air charging technology.

## GPS Loss

To make simulate losing and regaining GPS information you can just stop/restart the GPS driver. This is done by running the `gpssim stop` and `gpssim start` commands on your SITL instance *pxh shell*.