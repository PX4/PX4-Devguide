# Gazebo 机型

本话题列出/展示 PX4 [Gazebo](../simulation/gazebo.md) 仿真支持的机型以及运行这些机型所需的 `make` 指令（在 **Fimeware** 目录下启动一个终端来运行这些指令）。

支持的机型种类包括：多旋翼、VTOL、尾座式 VTOL、固定翼、无人车、潜艇/无人水下航行器。

> **Tip** 使用命令 `make px4_sitl list_vmd_make_targets` 获取构建目标的完整列表（你还可以过滤掉以 `gazebo_` 开头的目标）。

<span></span>
> **Note** [Gazebo](../simulation/gazebo.md) 页面展示了如何安装 Gazebo、如何启用视频并加载自定义地图以及许多其他的配置选项。

## Multicopter
### Quadrotor (Default) {#quadrotor}

```sh
make px4_sitl gazebo
```

### Quadrotor with Optical Flow {#quadrotor_optical_flow}

```sh
make px4_sitl gazebo_iris_opt_flow
```

### 3DR Solo (Quadrotor) {#3dr_solo}

```sh
make px4_sitl gazebo_solo
```

![3DR Solo 的 Gazebo 仿真](../../assets/simulation/gazebo/vehicles/solo.png)


### Typhoon H480 (Hexrotor) {#typhoon_h480}

```
make px4_sitl gazebo_typhoon_h480
```

![Typhoon H480 的 Gazebo 仿真](../../assets/simulation/gazebo/vehicles/typhoon.jpg)

> **Note** 此机型还支持[视频流仿真](#video)。

## Plane/Fixed Wing {#fixed_wing}

### Standard Plane {#standard_plane}

```sh
make px4_sitl gazebo_plane
```

![固定翼的 Gazebo 仿真](../../assets/simulation/gazebo/vehicles/plane.png)

## VTOL
### Standard VTOL {#standard_vtol}

```sh
make px4_sitl gazebo_standard_vtol
```

![标准 VTOL 的 Gazebo 仿真](../../assets/simulation/gazebo/vehicles/standard_vtol.png)

### Tailsitter VTOL {#tailsitter_vtol}

```sh
make px4_sitl gazebo_tailsitter
```

![尾座式 VTOL 的 Gazebo 仿真](../../assets/simulation/gazebo/vehicles/tailsitter.png)


## Unmmanned Ground Vehicle (UGV/Rover/Car) {#ugv}

### Ackerman UGV {#ugv_ackerman}

```sh
make px4_sitl gazebo_rover
```

![无人车的 Gazebo 仿真](../../assets/simulation/gazebo/vehicles/rover.png)

### Differential UGV {#ugv_differential}

```sh
make px4_sitl gazebo_r1_rover
```

![Rover in Gazebo](../../assets/simulation/gazebo/vehicles/r1_rover.png)


## Unmanned Underwater Vehicle (UUV/Submarine) {#uuv}

### HippoCampus TUHH UUV {#uuv_hippocampus}

```sh
make px4_sitl gazebo_uuv_hippocampus
```

![Submarine/UUV](../../assets/simulation/gazebo/vehicles/hippocampus.png)

## Unmanned Surface Vehicle (USV/Boat) {#usv}

### Boat {#usv_boat}

```sh
make px4_sitl gazebo_boat
```

![Boat/USV](../../assets/simulation/gazebo/vehicles/boat.png)
