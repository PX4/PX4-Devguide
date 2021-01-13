!REDIRECT "https://docs.px4.io/master/ko/"

# PX4 개발 안내서 ({{ book.px4_version }})

[![릴리즈](https://img.shields.io/badge/release-{{ book.px4_version }}-blue.svg)](https://github.com/PX4/PX4-Autopilot/releases) [![토의](https://img.shields.io/badge/discuss-px4-ff69b4.svg)](http://discuss.px4.io/) [![슬랙](https://px4-slack.herokuapp.com/badge.svg)](http://slack.px4.io)

> **Warning** This guide has been [merged into the PX4 User Guide](http://localhost:8080/px4_user_guide/en/development/development.html). It is now frozen for contributions and may be out of date. Please make any further edits, updates and translations in the user guide.

<span></span>

> **Info** This guide is for primarily for software developers and (new) hardware integrators. To fly, build and modify vehicles using supported hardware see the [PX4 User Guide](https://docs.px4.io/master/en/).

This guide explains how to:

* [최소 개발자 설정](setup/config_initial.md), [PX4 소스 코드 빌드](setup/building_px4.md), [다양하게 지원하는 오토파일럿](https://docs.px4.io/master/en/flight_controller/)으로의 펌웨어 배포 방법을 배웁니다.
* [PX4 시스템 구조](concept/architecture.md)와 다른 핵심 개념을 이해합니다.
* 플라이트 스택과 미들웨어 수정 방법을 배웁니다: 
  * 비행 알고리즘 수정 및 새 [비행 상태](concept/flight_modes.md) 추가.
  * 새 [에어프레임](airframes/README.md)을 지원합니다.
* PX4에 새 하드웨어를 붙이는 방법을 배웁니다: 
  * 카메라, 범위 검색 센서 등과 같은 새 센서, 액츄에이터를 지원합니다.
  * PX4를 수정하여 새 오토파일럿 하드웨어에서 구동합니다.
* PX4를 [모의시험](simulation/README.md), [실제 시험](test_and_ci/README.md), [오류 탐색/동작 기록](debug/README.md) 합니다.
* 외부 로보틱스 API와 통신/통합합니다.

## 지원

[Support](contribute/support.md) provide links to the [discussion boards](http://discuss.px4.io/) and other support channels.

> **Tip** The [Weekly Dev Call](contribute/dev_call.md) is another great opportunity to meet the PX4 dev team and discuss platform technical details, pull requests, major impacting issues etc. There is also time for Q&A.

## 기여 

[Contributing](contribute/README.md) explains how to work with our [source codelines](contribute/code.md), [documentation](contribute/docs.md), [translations](contribute/translation.md), and [licenses](contribute/licenses.md).

## 라이선스

The code is free to use and modify under terms of the permissive [BSD 3-clause license](https://opensource.org/licenses/BSD-3-Clause). The documentation is licensed under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). For more information see: [Licences](contribute/licenses.md).

## 번역

There are Chinese and Korean [translations](contribute/docs.md#translation) of this guide. You can access these by clicking the language-switcher icon:

![Gitbook Language Selector](../assets/gitbook/gitbook_language_selector.png)

<a id="calendar"></a>

## Calendar & Events

The *Dronecode Calendar* shows important events for platform developers and users. Select the links below to display the calendar in your timezone (and to add it to your own calendar):

* [스위스 – 취리히](https://calendar.google.com/calendar/embed?src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&ctz=Europe%2FZurich)
* [태평양 시간대 – 티후아나](https://calendar.google.com/calendar/embed?src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&ctz=America%2FTijuana)
* [오스트레일리아 – 멜버른/시드니/호바트](https://calendar.google.com/calendar/embed?src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&ctz=Australia%2FSydney)

**Note:** calendar defaults to CET.

{% raw %} <iframe src="https://calendar.google.com/calendar/embed?title=Dronecode%20Calendar&amp;mode=WEEK&amp;height=600&amp;wkst=1&amp;bgcolor=%23FFFFFF&amp;src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&amp;color=%23691426&amp;ctz=Europe%2FZurich" style="border-width:0" width="800" height="600" frameborder="0" scrolling="no" mark="crwd-mark"></iframe> 

{% endraw %}

## 운영

The PX4 flight stack is hosted under the governance of the [Dronecode Project](https://www.dronecode.org/).

<a href="https://www.dronecode.org/" style="padding:20px"><img src="https://mavlink.io/assets/site/logo_dronecode.png" alt="Dronecode Logo" width="110px"/></a>
<a href="https://www.linuxfoundation.org/projects" style="padding:20px;"><img src="https://mavlink.io/assets/site/logo_linux_foundation.png" alt="Linux Foundation Logo" width="80px" /></a>

<div style="padding:10px">&nbsp;</div>
