# 가제보 월드(world)

This topic provides imagery/information about the [Gazebo](../simulation/gazebo.md) worlds supported by PX4.

The [empty.world](#empty_world) is spawned by default, though this may be overridden by a [model specific world](#model_specific_worlds). Developers can also manually specify the world to load: [Gazebo Simulation > Loading a Specific World](../simulation/gazebo.md#set_world).

The source code for supported worlds can be found on GitHub here: [PX4/sitl_gazebo/worlds](https://github.com/PX4/sitl_gazebo/tree/master/worlds).

## 빈 월드 (기본) {#empty_world}

[PX4/sitl_gazebo/worlds/empty.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/empty.world)

![empty](../../assets/simulation/gazebo/worlds/empty.png)

## Baylands

[PX4/sitl_gazebo/worlds/baylands.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/baylands.world)

![Baylands World](../../assets/simulation/gazebo/worlds/baylands.jpg)

## KSQL 공항

[PX4/sitl_gazebo/worlds/ksql_airport.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/ksql_airport.world)

![KSQL Airport World](../../assets/simulation/gazebo/worlds/ksql_airport.jpg)

## McMillan Airfield

[PX4/sitl_gazebo/worlds/mcmillan_airfield.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/mcmillan_airfield.world)

![McMillan Airfield World](../../assets/simulation/gazebo/worlds/mcmillan_airfield.jpg)

## Sonoma Raceway

[PX4/sitl_gazebo/worlds/sonoma_raceway.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/sonoma_raceway.world) ![Sonoma_Raceway](../../assets/simulation/gazebo/worlds/sonoma_raceway.png)

## Warehouse

[PX4/sitl_gazebo/worlds/warehouse.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/warehouse.world)

![Warehouse](../../assets/simulation/gazebo/worlds/warehouse.png)

## Yosemite

[PX4/sitl_gazebo/worlds/yosemite.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/yosemite.world)

![Yosemite](../../assets/simulation/gazebo/worlds/yosemite.jpg)

## 모델별 월드 {#model_specific_worlds}

일부 [기체 모델](../simulation/gazebo_vehicles.md)은 특정 월드의 물리 구현체 / 플러그인에 따릅니다. PX4 툴체인에서는 기체 모델과 동일한 이름을 가진 월드가 있다면 (**empty.world**를 불러오지 않고) 자동으로 월드를 불러옵니다:

모델별 월드는 다음과 같습니다:
- [boat.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/boat.world): [boat](../simulation/gazebo_vehicles.md#usv)의 부력을 모의시험할 표면 정보가 들어있습니다..
- [uuv_hippocampus.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/uuv_hippocampus.world): [HippoCampus UUV](../simulation/gazebo_vehicles.md#uuv) 수중 환경 모의 시험용 빈 월드.
- [typhoon_h480.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/typhoon_h480.world): Used by [Typhoon H480 (Hexrotor)](../simulation/gazebo_vehicles.md#typhoon_h480) vehicle model and includes a video widget to enable / disable video streaming. The world includes a gazebo plugin for a simulated camera.
- [iris_irlock.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/iris_irlock.world): Includes a IR beacon for testing [precision landing](https://docs.px4.io/master/en/advanced_features/precland.html).
