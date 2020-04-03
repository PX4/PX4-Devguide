# Gazebo Vehicles

This topic lists/displays the vehicles supported by the PX4 [Gazebo](../simulation/gazebo.md) simulation and the `make` commands required to run them (the commands are run from a terminal in the **Firmware** directory).

Supported vehicle types include: mutirotors, VTOL, VTOL Tailsitter, Plane, Rover, Submarine/UUV.

> **Tip** For the full list of build targets run `make px4_sitl list_vmd_make_targets` (and filter on those that start with `gazebo_`).

<span></span>
> **Note** The [Gazebo](../simulation/gazebo.md) page shows how to install Gazebo, how to enable video and load custom maps, and many other configuration options.


## Quadrotor (Default) {#quadrotor}

```sh
make px4_sitl gazebo
```

## Quadrotor with Optical Flow {#quadrotor_optical_flow}

```sh
make px4_sitl gazebo_iris_opt_flow
```

## 3DR Solo (Quadrotor) {#3dr_solo}

```sh
make px4_sitl gazebo_solo
```

![3DR Solo in Gazebo](../../assets/simulation/gazebo/vehicles/solo.png)


## Typhoon H480 (Hexrotor) {#typhoon_h480}

```
make px4_sitl gazebo_typhoon_h480
```

![Typhoon H480 in Gazebo](../../assets/simulation/gazebo/vehicles/typhoon.jpg)

> **Note** This target also supports [video streaming simulation](#video).

## Standard Plane {#standard_plane}

```sh
make px4_sitl gazebo_plane
```

![Plane in Gazebo](../../assets/simulation/gazebo/vehicles/plane.png)

## Standard VTOL {#standard_vtol}

```sh
make px4_sitl gazebo_standard_vtol
```

![Standard VTOL in Gazebo](../../assets/simulation/gazebo/vehicles/standard_vtol.png)

## Tailsitter VTOL {#tailsitter_vtol}

```sh
make px4_sitl gazebo_tailsitter
```

![Tailsitter VTOL in Gazebo](../../assets/simulation/gazebo/vehicles/tailsitter.png)

## Ackerman vehicle (UGV/Rover) {#ugv}

```sh
make px4_sitl gazebo_rover
```

![Rover in Gazebo](../../assets/simulation/gazebo/vehicles/rover.png)


## HippoCampus TUHH (UUV: Unmanned Underwater Vehicle) {#uuv}

```sh
make px4_sitl gazebo_uuv_hippocampus
```

![Submarine/UUV](../../assets/simulation/gazebo/vehicles/hippocampus.png)

## Boat (USV: Unmanned Surface Vehicle) {#usv}

```sh
make px4_sitl gazebo_boat
```

![Boat/USV](../../assets/simulation/gazebo/vehicles/boat.png)

