# JMAVSim 进行多机仿真

本主题介绍如何使用 JMAVSim 仿真器配合软件在环仿真实现多机仿真。 在仿真中所有无人机实例均在同一位置启动。

> **Tip** 这是使用 PX4 进行多机仿真最简单的方法。 它适用于测试 *QGroundControl* （或者是 [MAVSDK](https://mavsdk.mavlink.io/)，等等 ） 对多机的支持。 [多机在 Gazebo 中的仿真](../simulation/multi-vehicle-simulation.md) 应用于多机进行无人机集群仿真，或者用于测试仅有 Gazebo 仿真平台支持的一些特性，比如计算机视觉。

<span></span>

> **Note** JMAVSim 多机仿真仅适用于 PX4 v1.8.0 及更高版本。

## 如何启动多机实例

要启动多个无人机实例 请执行以下操作（每架无人机使用一个单独的端口）：

1. 编译 PX4 ```make px4_sitl_default```
2. 运行 **sitl_multiple_run.sh**, 指定要启动的飞行器的实例数目 (例如 2): ```./Tools/sitl_multiple_run.sh 2```
3. 启动第一个实例: ```./Tools/jmavsim_run.sh -l```
4. 启动之后的实例，给实例指定的 *仿真* TCP 端口号： ```./Tools/jmavsim_run.sh -p 4561 -l``` 端口号应该被设置为 `4560+i` ， `i` 的范围为 `[0, N-1]` 。

像 *QGroundControl* 这样的地面站通过使用 UDP 14550 （所有流量都连接到同一端口） 端口来连接所有实例的。

像 *MAVSDK* 或者 *MAVROS* 开发者 APIs 接口就是通过连接 UDP 接口 14540 （第一个实例）， UDP 接口 14541（第二个实例），以此类推。

## 其他资源

* 看 [仿真](../simulation/README.md) 接口配置的更多信息。