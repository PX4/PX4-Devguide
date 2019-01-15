# PX4 Development Guide ({{ book.px4_version }})

[![Releases](https://img.shields.io/badge/release-{{ book.px4_version }}-blue.svg)](https://github.com/PX4/Firmware/releases) [![Discuss](https://img.shields.io/badge/discuss-px4-ff69b4.svg)](http://discuss.px4.io/) [![Slack](https://px4-slack.herokuapp.com/badge.svg)](http://slack.px4.io)

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
* [Simular](simulation/README.md), [probar](test_and_ci/README.md) y [depurar](debug/README.md) PX4.
* Comunicar/integrar con APIs externas.

## Foros y Chat {#support}

El equipo principal de desarrollo y la comunidad son activas en los siguientes foros y canales de chat.

* [PX4 Discuss](http://discuss.px4.io/) (*recomendado*)
* [Slack](http://slack.px4.io) (regístrate)
* [Google+](https://plus.google.com/117509651030855307398)

> **Nota** Los desarrolladores que quieran [aportar](contribute/README.md) a la plataforma son también bienvenidos a asistir a la [dev call semanal](contribute/README.md#dev_call) y a nuestros otros [eventos para desarrolladores](contribute/README.md#calendar).

## Aportaciones

[Aportaciones & Dev Call](contribute/README.md) explica cómo trabajar con nuestro código fuente. [Documentación](contribute/docs.md) explica cómo y dónde se pueden hacer cambios en la documentación.

## Traducciones

Hay [traducciones](contribute/docs.md#translation) del inglés, chino y coreano de esta guía. You can access these by clicking the language-switcher icon:

![Gitbook Language Selector](../assets/gitbook/gitbook_language_selector.png)

## Licencia

El código es libre de usar y modificar bajo términos de la [cláusula de licencia BSD 3](https://opensource.org/licenses/BSD-3-Clause). La documentación está registrada a través de [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). Para más información ver: [Licencias](contribute/licenses.md).

## Gobierno

El flight stack de PX4 está gestiónado a través del [Proyecto Dronecode](https://www.dronecode.org/).

<a href="https://www.dronecode.org/" style="padding:20px"><img src="https://mavlink.io/assets/site/logo_dronecode.png" alt="Dronecode Logo" width="110px"/></a>
<a href="https://www.linuxfoundation.org/projects" style="padding:20px;"><img src="https://mavlink.io/assets/site/logo_linux_foundation.png" alt="Logotipo de la Fundación de Linux" width="80px" /></a>

<div style="padding:10px">&nbsp;</div>
