# 가제보 기체

이 주제에서는 PX4 [가제보](../simulation/gazebo.md) 시뮬레이션에서 지원하는 기체와 기체를 가동하는데 필요한 `make` 명령을 나열합니다(명령은 **Firmware** 디렉터리 위치에서 터미널에서 실행).

지원 기체 형식은 멀티로터, 수직 이착륙기, 수직 이착륙 테일시터, 항공기, 탐사선, 수중선/무인 수중선이 있습니다.

> **Tip** 전체 대상을 빌드하려면 `make px4_sitl list_vmd_make_targets` 명령을 실행 (하고 `gazebo_`로 시작하는 요소를 검색) 하십시오.

<span></span>
> **Note** [가제보](../simulation/gazebo.md) 페이지에서는 가제보 설치 방법, 동영상 활성, 개별 지도 불러오기, 기타 설정 옵션을 보여줍니다.

## 멀티콥터
### 쿼드로터 (기본) {#quadrotor}

```sh
make px4_sitl gazebo
```

### 광류 센서 장착 쿼드로터 {#quadrotor_optical_flow}

```sh
make px4_sitl gazebo_iris_opt_flow
```

### 3DR 솔로 (쿼드로터) {#3dr_solo}

```sh
make px4_sitl gazebo_solo
```

![3DR Solo in Gazebo](../../assets/simulation/gazebo/vehicles/solo.png)


### 태풍 H480 (헥스로터) {#typhoon_h480}

```
make px4_sitl gazebo_typhoon_h480
```

![Typhoon H480 in Gazebo](../../assets/simulation/gazebo/vehicles/typhoon.jpg)

> **Note** 이 대상에서는[동영상 스트리밍 모의 시험환경](#video)도 지원합니다.

## 비행체/고정익 {#fixed_wing}

### 표준 비행체 {#standard_plane}

```sh
make px4_sitl gazebo_plane
```

![Plane in Gazebo](../../assets/simulation/gazebo/vehicles/plane.png)


#### 캐터펄트 발사형 표준 비행체 {#standard_plane_catapult}

```sh
make px4_sitl gazebo_plane_catapult
```

이 모델로 자세 준비 모드, 이륙 모드, 임무 비행시 [고정익 이륙](http://docs.px4.io/master/en/flying/fixed_wing_takeoff.html#fixed-wing-takeoff)시 손으로 날리거나 발사대로 날리는 비행체를 모의 시험 실시할 수 있습니다.

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
make px4_sitl gazebo_r1_rover
```

![가제보의 탐사선](../../assets/simulation/gazebo/vehicles/r1_rover.png)


## 무인 수중 기체(UUV/잠수함) {#uuv}

### 히포캠퍼스 TUHH UUV {#uuv_hippocampus}

```sh
make px4_sitl gazebo_uuv_hippocampus
```

![Submarine/UUV](../../assets/simulation/gazebo/vehicles/hippocampus.png)

## 무인 수면 기체(USV/보트) {#usv}

### 보트 {#usv_boat}

```sh
make px4_sitl gazebo_boat
```

![Boat/USV](../../assets/simulation/gazebo/vehicles/boat.png)

## 비행선 {#airship}

### 구름선 {#cloudship}

```sh
make px4_sitl gazebo_cloudship
```

![Airship](../../assets/simulation/gazebo/vehicles/airship.png)
