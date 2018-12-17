# Guía de Desarrollo de PX4

[![Releases](https://img.shields.io/github/release/PX4/Firmware.svg)](https://github.com/PX4/Firmware/releases) [![Discuss](https://img.shields.io/badge/discuss-px4-ff69b4.svg)](http://discuss.px4.io/) [![Slack](https://px4-slack.herokuapp.com/badge.svg)](http://slack.px4.io)

> **Información** Esta guía es principalmente para los desarrolladores de software e integradores de hardware. Para volar, construir y modificar vehículos usando el hardware soportado ver la [Guía del usuario de PX4](https://docs.px4.io/en/).

Esta guía explica cómo:

* Obtener una [configuración minima de desarrollo](setup/config_initial.md), [compilar el código fuente de PX4 ](setup/building_px4.md) y desplegar en los [numerosos autopilotos soportados](https://docs.px4.io/en/flight_controller/).
* Entender la [Arquitectura del sistema PX4](concept/architecture.md) y otros conceptos básicos.
* Aprender a modificar el flight stack y el middleware: 
  * Modificar algoritmos de vuelo y añadir nuevos [modos de vuelo](concept/flight_modes.md).
  * Soportar nuevos [fuselajes](airframes/README.md).
* Integrar PX4 en un nuevo hardware: 
  * Soportar nuevos sensores y actuadores, incluyendo cámaras, telémetros, etc.
  * Modificar PX4 para que funcione en un nuevo hardware autopiloto.
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

There are Chinese and Korean [translations](contribute/docs.md#translation) of this guide. You can you can access these by clicking the language-switcher icon:

![Gitbook Language Selector](../assets/gitbook/gitbook_language_selector.png)

## Licence

The code is free to use and modify under terms of the permissive [BSD 3-clause license](https://opensource.org/licenses/BSD-3-Clause). The documentation is licensed under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). For more information see: [Licences](contribute/licenses.md).

## Governance

The PX4 flight stack is hosted under the governance of the [Dronecode Project](https://www.dronecode.org/).

<a href="https://www.dronecode.org/" style="padding:20px"><img src="https://mavlink.io/assets/site/logo_dronecode.png" alt="Dronecode Logo" width="110px"/></a>
<a href="https://www.linuxfoundation.org/projects" style="padding:20px;"><img src="https://mavlink.io/assets/site/logo_linux_foundation.png" alt="Linux Foundation Logo" width="80px" /></a>

<div style="padding:10px">&nbsp;</div>
