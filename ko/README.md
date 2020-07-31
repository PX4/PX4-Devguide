# PX4 개발 안내서 ({{ book.px4_version }})

[![릴리즈](https://img.shields.io/badge/release-{{ book.px4_version }}-blue.svg)](https://github.com/PX4/Firmware/releases) [![토의](https://img.shields.io/badge/discuss-px4-ff69b4.svg)](http://discuss.px4.io/) [![슬랙](https://px4-slack.herokuapp.com/badge.svg)](http://slack.px4.io)

> **정보** 이 안내서는 초보 프로그램 개발자와 (새) 하드웨어 통합 작업자를 대상으로 합니다. 지원하는 하드웨어로 비행체를 날리고 구성하고 뜯어 고쳐보려면 [PX4 사용자 안내서](https://docs.px4.io/master/en/)를 확인하십시오.

이 안내서는 다음을 설명합니다:

* [최소 개발자 설정](setup/config_initial.md), [PX4 소스 코드 빌드](setup/building_px4.md), [다양하게 지원하는 오토파일럿](https://docs.px4.io/master/en/flight_controller/)으로의 펌웨어 배포 방법을 배웁니다.
* [PX4 시스템 구조](concept/architecture.md)와 다른 핵심 개념을 이해합니다.
* 플라이트 스택과 미들웨어 수정 방법을 배웁니다: 
  * 비행 알고리즘 수정 및 새 [비행 상태](concept/flight_modes.md) 추가.
  * 새 [airframes](airframes/README.md)을 지원합니다.
* PX4에 새 하드웨어를 붙이는 방법을 배웁니다: 
  * 카메라, 범위 검색 센서 등과 같은 새 센서, 액츄에이터를 지원합니다.
  * PX4를 수정하여 새 오토파일럿 하드웨어에서 구동합니다.
* PX4를 [모의시험](simulation/README.md), [실제 시험](test_and_ci/README.md), [오류 탐색/동작 기록](debug/README.md) 합니다.
* Communicate/integrate with external robotics APIs.

## 지원 {#support}

[지원](contribute/support.md) 페이지에서는 [토의 게시판](http://discuss.px4.io/)과 기타 지원 채널로의 링크를 제공합니다.

> **Tip** The [Weekly Dev Call](contribute/dev_call.md) is another great opportunity to meet the PX4 dev team and discuss platform technical details, pull requests, major impacting issues etc. There is also time for Q&A.

## 기여

[Contributing](contribute/README.md) explains how to work with our [source codelines](contribute/code.md), [documentation](contribute/docs.md), [translations](contribute/translation.md), and [licenses](contribute/licenses.md).

## 라이선스

코드는 [BSD 3-clause 라이선스](https://opensource.org/licenses/BSD-3-Clause)의 조항에 따라 자유롭게 사용하고 수정할 수 있습니다. 문서는 [크리에이티브 커먼즈 저작자표시 4.0](https://creativecommons.org/licenses/by/4.0/) 라이선스를 따릅니다. 자세한 정보는 [라이선스](contribute/licenses.md)를 참고하십시오.

## 번역

이 안내서의 중국어와 한국어 [번역](contribute/docs.md#translation)이 있습니다. 언어 전환 아이콘을 누르면 해당 언어 페이지에 접근할 수 있습니다:

![Gitbook 언어 선택](../assets/gitbook/gitbook_language_selector.png)

## 달력과 행사 {#calendar}

The *Dronecode Calendar* shows important events for platform developers and users. Select the links below to display the calendar in your timezone (and to add it to your own calendar):

* [Switzerland – Zurich](https://calendar.google.com/calendar/embed?src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&ctz=Europe%2FZurich)
* [Pacific Time – Tijuana](https://calendar.google.com/calendar/embed?src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&ctz=America%2FTijuana)
* [Australia – Melbourne/Sydney/Hobart](https://calendar.google.com/calendar/embed?src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&ctz=Australia%2FSydney)

**Note:** calendar defaults to CET.

{% raw %} <iframe src="https://calendar.google.com/calendar/embed?title=Dronecode%20Calendar&amp;mode=WEEK&amp;height=600&amp;wkst=1&amp;bgcolor=%23FFFFFF&amp;src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&amp;color=%23691426&amp;ctz=Europe%2FZurich" style="border-width:0" width="800" height="600" frameborder="0" scrolling="no" mark="crwd-mark"></iframe> 

{% endraw %}

## 운영

PX4 플라이트 기술 스택은 [드론코드 프로젝트](https://www.dronecode.org/)의 운영 기반으로 제공합니다.

<a href="https://www.dronecode.org/" style="padding:20px"><img src="https://mavlink.io/assets/site/logo_dronecode.png" alt="드론코드 로고" width="110px"/></a>
<a href="https://www.linuxfoundation.org/projects" style="padding:20px;"><img src="https://mavlink.io/assets/site/logo_linux_foundation.png" alt="리눅스 재단 로고" width="80px" /></a>

<div style="padding:10px">&nbsp;</div>
