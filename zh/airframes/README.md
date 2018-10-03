# 机架

PX4有一个灵活的[混合系统](../concept/mixing.md), 它允许通过单个代码库支持任何可以想象到的飞行器类型/帧:

* **固定翼飞机:**普通飞机, 飞翼, 倒V尾飞机等。
* **多旋翼:**直升机,三轴,四轴,六轴,dodecarotors等许多不同的几何图形。
* **VTOL Airframes:** VTOL configurations including: Tailsitters, Tiltrotors, and QuadPlanes (plane + quad).
* **UGVs/Rovers:** Basic support has been added for Unmanned Ground Vehicles, enabling both manual and mission-based control.

You can find a list of all supported frame types and motor outputs in the [Airframes Reference](../airframes/airframe_reference.md).

This section provides information that is relevant to developers who want to add support for new vehicles or vehicle types to PX4, including build logs for vehicles that are still being developed.

> **Tip** PX4 is also well-suited for use in other vehicle types and general robots, ranging from submarine, boats, or amphibious vehicles, through to experimental aircraft and rockets. *Let us know* if you have a new vehicle or frame-type you want to help support in PX4.

<span></span>

> **Note** Build logs for some of the supported airframes can be found in [PX4 User Guide > Airframes](https://docs.px4.io/en/airframes/).