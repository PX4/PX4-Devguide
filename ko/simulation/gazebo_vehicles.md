# 가제보 기체

This topic lists/displays the vehicles supported by the PX4 [Gazebo](../simulation/gazebo.md) simulation and the `make` commands required to run them (the commands are run from a terminal in the **Firmware** directory).

지원 기체 형식은 멀티로터, 수직 이착륙기, 수직 이착륙 테일시터, 항공기, 탐사선, 수중선/무인 수중선이 있습니다.

> **Tip** For the full list of build targets run `make px4_sitl list_vmd_make_targets` (and filter on those that start with `gazebo_`).

<span></span>
> **Note** The [Gazebo](../simulation/gazebo.md) page shows how to install Gazebo, how to enable video and load custom maps, and many other configuration options.

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

![3DR Solo in Gazebo](../../assets/simulation/gazebo/vehicles/solo.png)


### Typhoon H480 (Hexrotor) {#typhoon_h480}

```
make px4_sitl gazebo_typhoon_h480
```

![Typhoon H480 in Gazebo](../../assets/simulation/gazebo/vehicles/typhoon.jpg)

> **Note** This target also supports [video streaming simulation](#video).

## Plane/Fixed Wing {#fixed_wing}

### Standard Plane {#standard_plane}

```sh
make px4_sitl gazebo_plane
```

![Plane in Gazebo](../../assets/simulation/gazebo/vehicles/plane.png)


#### Standard Plane with Catapult Launch {#standard_plane_catapult}

```sh
make px4_sitl gazebo_plane_catapult
```

This model simulates hand/catapult launch, which can be used for [fixed wing takeoff](http://docs.px4.io/master/en/flying/fixed_wing_takeoff.html#fixed-wing-takeoff) in position mode, takeoff mode, or missions.

The plane will automatically be launched as soon as the vehicle is armed.


## 수직 이착륙기

### 표준 수직 이착륙기 {#standard_vtol}

```sh
make px4_sitl gazebo_standard_vtol
```

![가제보 표준 수직 이착륙기](../../assets/simulation/gazebo/vehicles/standard_vtol.png)

### 테일시터 수직 이착륙기 {#tailsitter_vtol}

```sh
make px4_sitl gazebo_tailsitter
```

![가제보 테일시터 수직 이착륙기](../../assets/simulation/gazebo/vehicles/tailsitter.png)


## 무인 지상 기체(UGV/탐사선/차량) {#ugv}

### Ackerman UGV {#ugv_ackerman}

```sh
make px4_sitl gazebo_rover
```

![가제보의 탐사선](../../assets/simulation/gazebo/vehicles/rover.png)

### Differential UGV {#ugv_differential}

```sh
Rover in Gazebo
```

![가제보의 탐사선](../../assets/simulation/gazebo/vehicles/r1_rover.png)


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

## Airship {#airship}

### Cloudship {#cloudship}

```sh
make px4_sitl gazebo_cloudship
```

![Airship](../../assets/simulation/gazebo/vehicles/airship.png)
