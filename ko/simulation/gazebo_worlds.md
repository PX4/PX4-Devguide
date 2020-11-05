# 가제보 월드(world)

This topic provides imagery/information about the [Gazebo](../simulation/gazebo.md) worlds supported by PX4.

The [empty.world](#empty_world) is spawned by default, though this may be overridden by a [model specific world](#model_specific_worlds). Developers can also manually specify the world to load: [Gazebo Simulation > Loading a Specific World](../simulation/gazebo.md#set_world).

The source code for supported worlds can be found on GitHub here: [PX4/sitl_gazebo/worlds](https://github.com/PX4/sitl_gazebo/tree/master/worlds).

<a id="empty_world"></a>

## Empty (Default)

[PX4/sitl_gazebo/worlds/empty.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/empty.world)

![빈 월드](../../assets/simulation/gazebo/worlds/empty.png)

## 베이랜드

[PX4/sitl_gazebo/worlds/baylands.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/baylands.world)

![베이랜드 월드](../../assets/simulation/gazebo/worlds/baylands.jpg)

## KSQL 공항

[PX4/sitl_gazebo/worlds/ksql_airport.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/ksql_airport.world)

![KSQL 공항 월드](../../assets/simulation/gazebo/worlds/ksql_airport.jpg)

## 맥밀리언 비행장

[PX4/sitl_gazebo/worlds/mcmillan_airfield.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/mcmillan_airfield.world)

![맥밀리언 비행장 월드](../../assets/simulation/gazebo/worlds/mcmillan_airfield.jpg)

## 소노마 경주로

[PX4/sitl_gazebo/worlds/sonoma_raceway.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/sonoma_raceway.world) ![소노마 경주로](../../assets/simulation/gazebo/worlds/sonoma_raceway.png)

## 창고

[PX4/sitl_gazebo/worlds/warehouse.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/warehouse.world)

![창고](../../assets/simulation/gazebo/worlds/warehouse.png)

## 요세미티

[PX4/sitl_gazebo/worlds/yosemite.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/yosemite.world)

![요세미티](../../assets/simulation/gazebo/worlds/yosemite.jpg)

<a id="model_specific_worlds"></a>

## Model Specific Worlds

일부 [기체 모델](../simulation/gazebo_vehicles.md)은 특정 월드의 물리 구현체 / 플러그인에 따릅니다. PX4 툴체인에서는 기체 모델과 동일한 이름을 가진 월드가 있다면 (**empty.world**를 불러오지 않고) 자동으로 월드를 불러옵니다:

모델별 월드는 다음과 같습니다:
- [boat.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/boat.world): [boat](../simulation/gazebo_vehicles.md#usv)의 부력을 모의시험할 표면 정보가 들어있습니다..
- [uuv_hippocampus.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/uuv_hippocampus.world): [HippoCampus UUV](../simulation/gazebo_vehicles.md#uuv) 수중 환경 모의 시험용 빈 월드.
- [typhoon_h480.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/typhoon_h480.world): [태풍 H480 (헥스로터)](../simulation/gazebo_vehicles.md#typhoon_h480) 기체 모델에서 활용하며 동영상 스트리밍 (비)/활성을 가능케 하는 동영상 위젯이 들어있습니다. 이 월드에는 카메라 모의시험용 카제보 플러그인이 들어있습니다.
- [iris_irlock.world](https://github.com/PX4/sitl_gazebo/blob/master/worlds/iris_irlock.world): [정밀 착륙](https://docs.px4.io/master/en/advanced_features/precland.html) 시험용 적외선 비콘이 들어있습니다.
