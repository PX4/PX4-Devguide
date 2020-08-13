# 초기 설정과 구성

개발자는 아래에 소개된 (또는 유사한) 기본 장비로 "기본" [에어프레임](../airframes/airframe_reference.md) 구성의 활용을 권장합니다.

## 기본 장비

> **Tip** PX4는 여기 설명된 것보다 훨씬 더 광범위한 장비 용도로 활용할 수 있지만, 초보 개발자는 표준 설정 중 하나를 사용하는 것이 좋습니다. Taranis 원격 조종 장치와 노트 4 태블릿 구성은 정말로 저렴한 현업용 키트입니다.

아래는 강력 추천 장비입니다:

* 안전 조종용 Taranis Plus 원격 조종기(또는 동급)
* 개발 컴퓨터: 
  * 맥북 프로(2015 초반 이후의 버전), macOS 10.15 이상 
  * 레노보 싱크패드 450 (i5), 우분투 리눅스 18.04 이상 
* 지상 통제국 장비: 
  * iPad(Wifi 텔레메트리 어댑터 필요)
  * 모든 MacBook 또는 Ubuntu Linux 랩톱(은 개발용 컴퓨터가 될 수 있음)
  * 삼성 Note 4 또는 동급 (*QGroundControl*를 효과적으로 실행하는 충분히 큰 화면을 갖는 모든 최신 Android 태블릿 또는 휴대폰)
* 보안경
* 보다 위험한 시험을 위한 멀티콥터용 끈

## 기체 구성

> **Tip** 기체 설정시 **데스크톱 운영체제**용 *QGroundControl* 프로그램이 필요합니다. PX4의 최신 기능을 이용하려면 일일 빌드를 사용(하고 주기적으로 업데이트) 해야합니다.

기체를 구성하려면:

1. 개발 플랫폼용 [QGroundControl 일일 빌드](https://docs.qgroundcontrol.com/en/releases/daily_builds.html)를 다운로드 하십시오.
2. [기본 설정](https://docs.px4.io/master/en/config/) (PX4 사용자 안내서)에서 기본 설정을 진행하는 방법을 설명합니다. 
3. [매개변수 설정](https://docs.px4.io/master/en/advanced_config/parameters.html) (PX4 사용자 안내서) 에서는 개별 매개변수를 찾아 설정하는 방법을 설명합니다.