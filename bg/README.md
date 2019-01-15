# PX4 Development Guide ({{ book.px4_version }})

[![Releases](https://img.shields.io/badge/release-{{ book.px4_version }}-blue.svg)](https://github.com/PX4/Firmware/releases) [![Discuss](https://img.shields.io/badge/discuss-px4-ff69b4.svg)](http://discuss.px4.io/) [![Slack](https://px4-slack.herokuapp.com/badge.svg)](http://slack.px4.io)

> **Info** This guide is for primarily for software developers and (new) hardware integrators. To fly, build and modify vehicles using supported hardware see the [PX4 User Guide](https://docs.px4.io/en/).

This guide explains how to:

* Get a [minimum developer setup](setup/config_initial.md), [build PX4 from source](setup/building_px4.md) and deploy on [numerous supported autopilots](https://docs.px4.io/en/flight_controller/).
* Understand the [PX4 System Architecture](concept/architecture.md) and other core concepts.
* Learn how to modify the flight stack and middleware: 
  * Modify flight algorithms and add new [flight modes](concept/flight_modes.md).
  * Support new [airframes](airframes/README.md).
* Learn how to integrate PX4 with new hardware: 
  * Support new sensors and actuators, including cameras, rangefinders, etc.
  * Modify PX4 to run on new autopilot hardware.
* [Simulate](simulation/README.md), [test](test_and_ci/README.md) and [debug/log](debug/README.md) PX4.
* Communicate/integrate with external robotics APIs.

## Forums and Chat {#support}

The core development team and community are active on the following forums and chat channels.

* [PX4 Discuss](http://discuss.px4.io/) (*recommended*)
* [Slack](http://slack.px4.io) (sign up)
* [Google+](https://plus.google.com/117509651030855307398)

> **Tip** Developers who want to [contribute](contribute/README.md) to the platform are also most welcome to attend the [weekly dev call](contribute/README.md#dev_call) and our other [developer events](contribute/README.md#calendar).

## Contributing

[Contributing & Dev Call](contribute/README.md) explains how to work with our source codelines. [Documentation](contribute/docs.md) explains how and where documentation changes can be made.

## Translations

There are Chinese and Korean [translations](contribute/docs.md#translation) of this guide. You can access these by clicking the language-switcher icon:

![Gitbook Language Selector](../assets/gitbook/gitbook_language_selector.png)

## Licence

The code is free to use and modify under terms of the permissive [BSD 3-clause license](https://opensource.org/licenses/BSD-3-Clause). The documentation is licensed under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). For more information see: [Licences](contribute/licenses.md).

## Governance

The PX4 flight stack is hosted under the governance of the [Dronecode Project](https://www.dronecode.org/).

<a href="https://www.dronecode.org/" style="padding:20px"><img src="https://mavlink.io/assets/site/logo_dronecode.png" alt="Dronecode Logo" width="110px"/></a>
<a href="https://www.linuxfoundation.org/projects" style="padding:20px;"><img src="https://mavlink.io/assets/site/logo_linux_foundation.png" alt="Linux Foundation Logo" width="80px" /></a>

<div style="padding:10px">&nbsp;</div>
