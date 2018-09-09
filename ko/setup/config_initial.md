# 초기 설정과 구성

개발자는 아래에 소개된 (또는 유사한) 기본 장비로 "기본" [에어프레임](../airframes/airframe_reference.md) 구성을 사용할 것을 권장합니다.

## 기본 장비

> **팁** PX4는 여기 설명된 것보다 훨씬 더 광범위한 장비로 사용되지만, 초보 개발자들은 표준 설정 중 하나를 사용하는 것이 좋습니다. 아주 저렴한 필트 키트로 Taranis RC와 노트 4 구성이 있습니다.

아래는 강력 추천 장비입니다:

* 안전 조종을 위해 Taranis Plus 원격 조종기 (또는 동급)
* 개발 컴퓨터: 
  * OSX 10.13 또는 그 이후 운영체제가 설치된 MacBook Pro (2015 초기 버전 이후)
  * 우분투 16.04 또는 그 이후 운영체제가 설치된 Lenovo Thinkpad 450 (i5)
* 지상통제소 장비: 
  * iPad(Wifi 텔레메트리 어댑터 필요)
  * 모든 MacBook 또는 Ubuntu Linux 랩톱(은 개발용 컴퓨터가 될 수 있음)
  * Samsung Note 4 or equivalent (any recent Android tablet or phone with a large enough screen to run *QGroundControl* effectively).
* Safety glasses
* For multicopters - tether for more risky tests

## Vehicle Configuration

> **Tip** *QGroundControl* for a **desktop OS** is required for vehicle configuration. You should use (and regularly update) the daily build in order to take advantage of the latest features in PX4.

To configure the vehicle:

1. Download the [QGroundControl Daily Build](https://docs.qgroundcontrol.com/en/releases/daily_builds.html) for your development platform.
2. [Basic Configuration](https://docs.px4.io/en/config/) (PX4 User Guide) explains how to to perform basic configuration. 
3. [Parameter Configuration](https://docs.px4.io/en/advanced_config/parameters.html) (PX4 User Guide) explains how you can find and modify individual parameters.