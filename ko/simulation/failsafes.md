# Failsafe 시뮬레이션하기

[Failsafe](https://docs.px4.io/master/en/config/safety.html)은 PX4를 안전하게 사용할 수 있는 안전 한계/조건과 failsafe가 트리거된 경우에 수행될 동작을 정의합니다. (예를 들면, 착륙, 위치 유지, 혹은 지정된 지점으로 복귀)

In SITL some failsafes are disabled by default to enable easier simulation usage. This topic explains how you can test safety-critical behavior in SITL simulation before attempting it in the real world.

> **Note** You can also test failsafes using [HITL simulation](../simulation/hitl.md). HITL uses the normal configuration parameters of your flight controller.

<span></span>

> **Tip** The [SITL parameters](../advanced/parameter_reference.md#sitl) allow you to simulate other common sensor failure cases that are not covered here, including: loss of barometer, gyro and accelerometer, increased GPS noise etc.

## Data Link Loss

The *Data Link Loss* failsafe (unavailability of external data via MAVLink) is enabled by default. This makes the simulation only usable with a connected GCS, SDK, or other MAVLink application.

Set the parameter [NAV_DLL_ACT](../advanced/parameter_reference.md#NAV_DLL_ACT) to the desired failsafe action to change the behavior. For example, set to `0` to disable it.

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

To simulate losing and regaining GPS information you can just stop the publication of GPS messages. This is done by running the `param set SIM_GPS_BLOCK 1` and `param set SIM_GPS_BLOCK 0` commands on your SITL instance *pxh shell* to block and unblock messages respectively.