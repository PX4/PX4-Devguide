# PX4 개발 안내서 ({{ book.px4_version }})

[![Releases](https://img.shields.io/badge/release-{{ book.px4_version }}-blue.svg)](https://github.com/PX4/Firmware/releases) [![Discuss](https://img.shields.io/badge/discuss-px4-ff69b4.svg)](http://discuss.px4.io/) [![Slack](https://px4-slack.herokuapp.com/badge.svg)](http://slack.px4.io)

> **정보** 이 안내서는 초보 프로그램 개발자와 (새) 하드웨어 통합 작업자를 대상으로 합니다. 지원하는 하드웨어로 비행체를 날리고 구성하고 뜯어 고쳐보려면 [PX4 사용자 안내서](https://docs.px4.io/master/en/)를 확인하십시오.

이 안내서는 다음을 설명합니다:

* [최소 개발자 설정](setup/config_initial.md), [PX4 소스 코드 빌드](setup/building_px4.md), [다양하게 지원하는 오토파일럿](https://docs.px4.io/master/en/flight_controller/)으로의 펌웨어 배포 방법을 배웁니다.
* [PX4 시스템 구조](concept/architecture.md)와 다른 핵심 개념을 이해합니다.
* 플라이트 스택과 미들웨어 수정 방법을 배웁니다: 
  * Modify flight algorithms and add new [flight modes](concept/flight_modes.md).
  * Support new [airframes](airframes/README.md).
* Learn how to integrate PX4 with new hardware: 
  * Support new sensors and actuators, including cameras, rangefinders, etc.
  * Modify PX4 to run on new autopilot hardware.
* [Simulate](simulation/README.md), [test](test_and_ci/README.md) and [debug/log](debug/README.md) PX4.
* Communicate/integrate with external robotics APIs.

## Support {#support}

[Support](contribute/support.md) provide links to the [discussion boards](http://discuss.px4.io/) and other support channels.

> **Tip** The [Weekly Dev Call](contribute/dev_call.md) is another great opportunity to meet the PX4 dev team and discuss platform technical details, pull requests, major impacting issues etc. There is also time for Q&A.

## Contributing

[Contributing](contribute/README.md) explains how to work with our [source codelines](contribute/code.md), [documentation](contribute/docs.md), [translations](contribute/translation.md), and [licenses](contribute/licenses.md).

## Licence

The code is free to use and modify under terms of the permissive [BSD 3-clause license](https://opensource.org/licenses/BSD-3-Clause). The documentation is licensed under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). For more information see: [Licences](contribute/licenses.md).

## Translations

There are Chinese and Korean [translations](contribute/docs.md#translation) of this guide. You can access these by clicking the language-switcher icon:

![Gitbook Language Selector](../assets/gitbook/gitbook_language_selector.png)

## Calendar & Events {#calendar}

The *Dronecode Calendar* shows important events for platform developers and users. Select the links below to display the calendar in your timezone (and to add it to your own calendar):

* [Switzerland – Zurich](https://calendar.google.com/calendar/embed?src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&ctz=Europe%2FZurich)
* [Pacific Time – Tijuana](https://calendar.google.com/calendar/embed?src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&ctz=America%2FTijuana)
* [Australia – Melbourne/Sydney/Hobart](https://calendar.google.com/calendar/embed?src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&ctz=Australia%2FSydney)

**Note:** calendar defaults to CET.

{% raw %} <iframe src="https://calendar.google.com/calendar/embed?title=Dronecode%20Calendar&amp;mode=WEEK&amp;height=600&amp;wkst=1&amp;bgcolor=%23FFFFFF&amp;src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&amp;color=%23691426&amp;ctz=Europe%2FZurich" style="border-width:0" width="800" height="600" frameborder="0" scrolling="no" mark="crwd-mark"></iframe> 

{% endraw %}

## Governance

The PX4 flight stack is hosted under the governance of the [Dronecode Project](https://www.dronecode.org/).

<a href="https://www.dronecode.org/" style="padding:20px"><img src="https://mavlink.io/assets/site/logo_dronecode.png" alt="Dronecode Logo" width="110px"/></a>
<a href="https://www.linuxfoundation.org/projects" style="padding:20px;"><img src="https://mavlink.io/assets/site/logo_linux_foundation.png" alt="Linux Foundation Logo" width="80px" /></a>

<div style="padding:10px">&nbsp;</div>
