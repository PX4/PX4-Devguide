# QGroundControl

QGroundControl는 PX4 기반 오토파일럿을 설정하고 운항하는 앱입니다. 여러 플랫폼과 운영체제를 지원합니다:

- 모바일: 안드로이드와 iOS(현재는 태블릿에 맞춤)
- 데스크톱: 윈도우, 리눅스, Mac OS

## 임무 계획

새 임무를 계획하려면, planning 탭으로 전환한 후, 좌측 상단의 + 아이콘을 눌러 경로점을 만들 준비를 하고 지도상에 경로점을 찍으십시오. 경로점을 편집할 단축 메뉴는 측면에 나타납니다. Click on the highlight transmission icon to send them to the vehicle.

![](../../assets/gcs/planning-mission.png)

## 임무 비행

Flying 탭으로 전환하십시오. 임무 내용이 지도에 나타나야합니다. MISSION으로 바꿀 현재 비행 모드를 누른 후, DISARMED를 눌러 기체의 비행 준비를 마치십시오. 비행체가 이미 비행 중에 있을 경우 임무 첫 지점으로 비행하며 그 다음 지점 경로를 따라갑니다.

![](../../assets/gcs/flying-mission.png)

## 매개변수 설정

설정 탭으로 전환하십시오. Scroll the menu on the left all the way to the bottom and click on the parameter icon. Parameters can be changed by double-clicking on them, which opens a context menu to edit, along with a more detailed description.

![](../../assets/gcs/setting-parameter.png)

## Installation

QGroundControl can be downloaded from its [website](http://qgroundcontrol.com/downloads).

> **Tip** Developers are advised to use the latest daily build instead of the stable release.

## Building from source

Firmware developers are encouraged to build from source in order to have a matching recent version to their flight code.

Follow the [QGroundControl build instructions](https://dev.qgroundcontrol.com/en/getting_started/) to install Qt and build the source code.