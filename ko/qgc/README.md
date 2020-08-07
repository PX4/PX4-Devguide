# QGroundControl

QGroundControl는 PX4 기반 오토파일럿을 설정하고 운항하는 앱입니다. 여러 플랫폼과 운영체제를 지원합니다:

- 모바일: 안드로이드와 iOS(현재는 태블릿에 맞춤)
- 데스크톱: 윈도우, 리눅스, Mac OS

## 임무 계획

새 임무를 계획하려면, planning 탭으로 전환한 후, 좌측 상단의 + 아이콘을 눌러 경로점을 만들 준비를 하고 지도상에 경로점을 찍으십시오. A context menu will open on the side to adjust the waypoints. Click on the highlight transmission icon to send them to the vehicle.

![](../../assets/gcs/planning-mission.png)

## Flying Missions

Switch to the flying tab. The mission should be visible on the map. Click on the current flight mode to change it to MISSION and click on DISARMED to arm the vehicle. If the vehicle is already in flight it will fly to the first leg of the mission and then follow it.

![](../../assets/gcs/flying-mission.png)

## Setting parameters

Switch to the setup tab. Scroll the menu on the left all the way to the bottom and click on the parameter icon. Parameters can be changed by double-clicking on them, which opens a context menu to edit, along with a more detailed description.

![](../../assets/gcs/setting-parameter.png)

## Installation

QGroundControl can be downloaded from its [website](http://qgroundcontrol.com/downloads).

> **Tip** Developers are advised to use the latest daily build instead of the stable release.

## Building from source

Firmware developers are encouraged to build from source in order to have a matching recent version to their flight code.

Follow the [QGroundControl build instructions](https://dev.qgroundcontrol.com/en/getting_started/) to install Qt and build the source code.