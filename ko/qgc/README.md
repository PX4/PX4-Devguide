!REDIRECT "https://docs.px4.io/master/ko/qgc/"

# QGroundControl

QGroundControl는 PX4 기반 오토파일럿을 설정하고 운항하는 앱입니다. 여러 플랫폼과 운영체제를 지원합니다:

- 모바일: 안드로이드와 iOS(현재는 태블릿에 맞춤)
- 데스크톱: 윈도우, 리눅스, Mac OS

## 임무 계획

새 임무를 계획하려면, planning 탭으로 전환한 후, 좌측 상단의 + 아이콘을 눌러 경로점을 만들 준비를 하고 지도상에 경로점을 찍으십시오. 경로점을 편집할 단축 메뉴는 측면에 나타납니다. 강조 상태의 전송 아이콘을 눌러 기체로 임무 데이터를 보내십시오.

![](../../assets/gcs/planning-mission.png)

## 임무 비행

Flying 탭으로 전환하십시오. 임무 내용이 지도에 나타나야합니다. MISSION으로 바꿀 현재 비행 모드를 누른 후, DISARMED를 눌러 기체의 비행 준비를 마치십시오. 비행체가 이미 비행 중에 있을 경우 임무 첫 지점으로 비행하며 그 다음 지점 경로를 따라갑니다.

![](../../assets/gcs/flying-mission.png)

## 매개변수 설정

설정 탭으로 전환하십시오. 좌측의 메뉴를 하단으로 내려 매개변수 아이콘을 누르십시오. 매개변수 값은 단축 메뉴에서 편집을 선택, 자세한 설명을 따라 편집하여 바꿀 수 있습니다.

![](../../assets/gcs/setting-parameter.png)

## 설치

QGroundControl은 [웹사이트](http://qgroundcontrol.com/downloads)에서 다운로드할 수 있습니다.

> **Tip** 개발자 여러분은 안정 릴리스보다 최신 일일 빌드를 받으시는게 좋습니다.

## 소스 코드 빌드

펌웨어 개발자는 비행 코드를 최신 버전에 맞추기 위해 소스 코드의 직접 빌드를 권장합니다.

[QGroundControl 빌드 절차](https://dev.qgroundcontrol.com/en/getting_started/)를 따라 Qt를 설치하고 소스 코드를 빌드하십시오.