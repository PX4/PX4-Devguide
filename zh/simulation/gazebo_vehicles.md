# Gazebo 机型

本话题列出/展示 PX4 [Gazebo](../simulation/gazebo.md) 仿真支持的机型以及运行这些机型所需的 `make` 指令（在 **Fimeware** 目录下启动一个终端来运行这些指令）。

支持的机型种类包括：多旋翼、VTOL、尾座式 VTOL、固定翼、无人车、潜艇/无人水下航行器。

> **Tip** 使用命令 `make px4_sitl list_vmd_make_targets` 获取构建目标的完整列表（你还可以过滤掉以 `gazebo_` 开头的目标）。

<span></span>
> **Note** [Gazebo](../simulation/gazebo.md) 页面展示了如何安装 Gazebo、如何启用视频并加载自定义地图以及许多其他的配置选项。


## 四旋翼（默认） {#quadrotor}

```sh
make px4_sitl gazebo
```

## 具有光流的四旋翼（#quadrotor_optical_flow）

```sh
make px4_sitl gazebo_iris_opt_flow
```

## 3DR Solo（四旋翼） {#3dr_solo}

```sh
make px4_sitl gazebo_solo
```

![3DR Solo 的 Gazebo 仿真](../../assets/simulation/gazebo/vehicles/solo.png)


## Typhoon H480（六旋翼） {#typhoon_h480}

```
make px4_sitl gazebo_typhoon_h480
```

![Typhoon H480 的 Gazebo 仿真](../../assets/simulation/gazebo/vehicles/typhoon.jpg)

> **Note** 此机型还支持[视频流仿真](#video)。

## 标准构型的固定翼 {#standard_plane}

```sh
make px4_sitl gazebo_plane
```

![固定翼的 Gazebo 仿真](../../assets/simulation/gazebo/vehicles/plane.png)

## 标准 VTOL {#standard_vtol}

```sh
make px4_sitl gazebo_standard_vtol
```

![标准 VTOL 的 Gazebo 仿真](../../assets/simulation/gazebo/vehicles/standard_vtol.png)

## 尾座式 VTOL {#tailsitter_vtol}

```sh
make px4_sitl gazebo_tailsitter
```

![尾座式 VTOL 的 Gazebo 仿真](../../assets/simulation/gazebo/vehicles/tailsitter.png)

## Ackerman 地面车辆（UGV/Rover） {#ugv}

```sh
make px4_sitl gazebo_rover
```

![无人车的 Gazebo 仿真](../../assets/simulation/gazebo/vehicles/rover.png)


## HippoCampus TUHH（UUV：无人水下航行器） {#uuv}

```sh
make px4_sitl gazebo_uuv_hippocampus
```

![潜艇/UUV 的 Gazebo 仿真](../../assets/simulation/gazebo/vehicles/hippocampus.png)

## 船（USV：无人驾驶水面艇） {#usv}

```sh
make px4_sitl gazebo_boat
```

![船/USV](../../assets/simulation/gazebo/vehicles/boat.png)

