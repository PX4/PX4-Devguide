# 가제보 기체

This topic lists/displays the vehicles supported by the PX4 [Gazebo](../simulation/gazebo.md) simulation and the `make` commands required to run them (the commands are run from a terminal in the **PX4-Autopilot** directory).

지원 기체 형식은 멀티로터, 수직 이착륙기, 수직 이착륙 테일시터, 항공기, 탐사선, 수중선/무인 수중선이 있습니다.

> **Tip** 전체 대상을 빌드하려면 `make px4_sitl list_vmd_make_targets` 명령을 실행 (하고 `gazebo_`로 시작하는 요소를 검색) 하십시오.

<span></span>
> **Note** [가제보](../simulation/gazebo.md) 페이지에서는 가제보 설치 방법, 동영상 활성, 개별 지도 불러오기, 기타 설정 옵션을 보여줍니다.

## 멀티콥터

<a id="quadrotor"></a>

### Quadrotor (Default)

```sh
make px4_sitl gazebo
```

<a id="quadrotor_optical_flow"></a>

### Quadrotor with Optical Flow

```sh
make px4_sitl gazebo_iris_opt_flow
```

<a id="3dr_solo"></a>

### 3DR Solo (Quadrotor)

```sh
make px4_sitl gazebo_solo
```

![가제보 3DR 솔로](../../assets/simulation/gazebo/vehicles/solo.png)

<a id="typhoon_h480"></a>

### Typhoon H480 (Hexrotor)

```
make px4_sitl gazebo_typhoon_h480
```

![가제보 태풍 H480](../../assets/simulation/gazebo/vehicles/typhoon.jpg)

> **Note** 이 대상에서는[동영상 스트리밍 모의 시험환경](#video)도 지원합니다.

<a id="fixed_wing"></a>

## Plane/Fixed Wing

<a id="standard_plane"></a>

### Standard Plane

```sh
make px4_sitl gazebo_plane
```

![가제보 비행체](../../assets/simulation/gazebo/vehicles/plane.png)

<a id="standard_plane_catapult"></a>

#### Standard Plane with Catapult Launch

```sh
make px4_sitl gazebo_plane_catapult
```

이 모델로 자세 준비 모드, 이륙 모드, 임무 비행시 [고정익 이륙](http://docs.px4.io/master/en/flying/fixed_wing_takeoff.html#fixed-wing-takeoff)시 손으로 날리거나 발사대로 날리는 비행체를 모의 시험 실시할 수 있습니다.

비행체는 기체 동력을 인가하고난 후 바로 자동으로 발사할 수 있습니다.


## 수직 이착륙기

<a id="standard_vtol"></a>

### Standard VTOL

```sh
make px4_sitl gazebo_standard_vtol
```

![가제보 표준 수직 이착륙기](../../assets/simulation/gazebo/vehicles/standard_vtol.png)

<a id="tailsitter_vtol"></a>

### Tailsitter VTOL

```sh
make px4_sitl gazebo_tailsitter
```

![가제보 테일시터 수직 이착륙기](../../assets/simulation/gazebo/vehicles/tailsitter.png)

<a id="ugv"></a>

## Unmmanned Ground Vehicle (UGV/Rover/Car)

<a id="ugv_ackerman"></a>

### Ackerman UGV

```sh
make px4_sitl gazebo_rover
```

![가제보의 탐사선](../../assets/simulation/gazebo/vehicles/rover.png)

<a id="ugv_differential"></a>

### Differential UGV

```sh
make px4_sitl gazebo_r1_rover
```

![가제보의 탐사선](../../assets/simulation/gazebo/vehicles/r1_rover.png)

<a id="uuv"></a>

## Unmanned Underwater Vehicle (UUV/Submarine)

<a id="uuv_hippocampus"></a>

### HippoCampus TUHH UUV

```sh
make px4_sitl gazebo_uuv_hippocampus
```

![잠수함/UUV](../../assets/simulation/gazebo/vehicles/hippocampus.png)

<a id="usv"></a>

## Unmanned Surface Vehicle (USV/Boat)

<a id="usv_boat"></a>

### Boat

```sh
make px4_sitl gazebo_boat
```

![보트/USV](../../assets/simulation/gazebo/vehicles/boat.png)

<a id="airship"></a>

## Airship

<a id="cloudship"></a>

### Cloudship

```sh
make px4_sitl gazebo_cloudship
```

![비행선](../../assets/simulation/gazebo/vehicles/airship.png)
