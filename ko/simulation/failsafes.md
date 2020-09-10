# Failsafe 시뮬레이션하기

[안전장치](https://docs.px4.io/master/en/config/safety.html)는 PX4를 안전하게 활용할 수 있는 안전 한계와 조건, 그리고 안전장치 가동을 개시했을 때 취할 수 있는 동작을 정의합니다(예: 착륙, 자세 유지 위치, 지정 지점으로의 복귀 등).

SITL에서는 모의시험 활용의 용이성을 위해 기본적으로 안전장치를 끕니다. 이 주제에서는 실제로 안전 장치를 가동해보기 전에 SITL 모의시험 환경에서 안전 위해 동작을 시험해볼 수 있는 방법을 설명합니다.

> **Note** [HITL 시뮬레이션](../simulation/hitl.md)을 활용해도 failsafe 테스트가 가능합니다. HITL uses the normal configuration parameters of your flight controller.

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