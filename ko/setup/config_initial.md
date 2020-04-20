# 초기 설정과 구성

개발자는 아래에 소개된 (또는 유사한) 기본 장비로 "기본" [에어프레임](../airframes/airframe_reference.md) 구성을 사용할 것을 권장합니다.

## 기본 장비

> **팁** PX4는 여기 설명된 것보다 훨씬 더 광범위한 장비로 사용되지만, 초보 개발자들은 표준 설정 중 하나를 사용하는 것이 좋습니다. 아주 저렴한 필트 키트로 Taranis RC와 노트 4 구성이 있습니다.

아래는 강력 추천 장비입니다:

* 안전 조종을 위해 Taranis Plus 원격 조종기 (또는 동급)
* 개발 컴퓨터: 
  * MacBook Pro (early 2015 and later) with OSX 10.15 or later 
  * Lenovo Thinkpad 450 (i5) with Ubuntu Linux 18.04 or later 
* 지상통제소 장비: 
  * iPad(Wifi 텔레메트리 어댑터 필요)
  * 모든 MacBook 또는 Ubuntu Linux 랩톱(은 개발용 컴퓨터가 될 수 있음)
  * 삼성 Note 4 또는 동급 (*QGroundControl*를 효과적으로 실행하는 충분히 큰 화면을 갖는 모든 최신 Android 태블릿 또는 휴대폰)
* 보안경
* 보다 위험한 시험을 위한 멀티콥터용 끈

## 비행기 구성

> **팁** **데스크탑용** *QGroundControl*이 비행기 구성에 필요합니다. PX4의 최신 기능을 이용하려면 데일리 빌드를 사용(하고 주기적으로 업데이트) 해야합니다.

비행기를 구성하려면:

1. 개발 플랫폼을 위해 [QGroundControl 데일리 빌드](https://docs.qgroundcontrol.com/en/releases/daily_builds.html)를 다운로드 하십시오.
2. [Basic Configuration](https://docs.px4.io/master/en/config/) (PX4 User Guide) explains how to to perform basic configuration. 
3. [Parameter Configuration](https://docs.px4.io/master/en/advanced_config/parameters.html) (PX4 User Guide) explains how you can find and modify individual parameters.