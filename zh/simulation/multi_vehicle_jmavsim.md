# 使用 JMAVSim 进行多飞行器仿真

本主题介绍如何使用 JMAVSim和 SITL 模拟多架 (多旋翼) 无人机。 在仿真中所有无人机实例均在同一位置启动。

> **Tip** 这是模拟多架运行 PX4 无人机的最简单的方法。 可用于测试 *QGroundControl*对多无人机的支持 (或 [Dronecode SDK](https://sdk.dronecode.org/en/)) 。 [Multi-Vehicle Simulation with Gazebo](../simulation/multi-vehicle-simulation.md) 应用于使用多架无人机进行无人机蜂群模拟，或者用于测试仅有 Gazebo 仿真平台支持的一些特性，比如计算机视觉。

<span></span>

> **Note** JMAVSim 多飞行器仿真仅适用于 PX4 v1.8.0 及更高版本。

## 如何启动多个飞行器实例

要启动多个无人机实例 请执行以下操作（每架无人机使用一个单独的端口）：

1. 编译 PX4 ```make px4_sitl_default```
2. 运行 **sitl_multiple_run.sh**, 指定要启动的飞行器的实例数目 (例如 2): ```./Tools/sitl_multiple_run.sh 2```
3. 启动第一个实例: ```./Tools/jmavsim_run.sh -l```
4. Start subsequent instances, specifying the *simulation* TCP port for the instance: ```./Tools/jmavsim_run.sh -p 4561 -l``` The port should be set to `4560+i` for `i` in `[0, N-1]`.

Ground stations such as *QGroundControl* connect to all instances using the normal UDP port 14550 (all traffic goes to the same port).

Developer APIs such as *Dronecode SDK* or *MAVROS* connect on the UDP port 14540 (first instance), UDP port 14541 (second instance), and so on.

## 额外资源

* See [Simulation](../simulation/README.md) for more information about the port configuration.