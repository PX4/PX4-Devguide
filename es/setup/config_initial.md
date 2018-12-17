# Configuración inicial & Configuración

Recomendamos que los desarrolladores obtengan el equipo básico descrito a continuación (o similar) y utilicen una configuración de [fuselaje](../airframes/airframe_reference.md) "default".

## Equipo básico

> **Nota** PX4 puede ser usado con una gama de equipos mucho más amplia que la que se describe aquí, pero los nuevos desarrolladores se beneficiarán de ir con una de las configuraciones estándar. Un RC de Taranis además de una tablet Note 4 constituyen un kit muy barato.

The equipment below is highly recommended:

* A Taranis Plus remote control for the safety pilot (or equivalent)
* A development computer: 
  * MacBook Pro (early 2015 and later) with OSX 10.13 or later
  * Lenovo Thinkpad 450 (i5) with Ubuntu Linux 16.04 or later
* A ground control station device: 
  * iPad (requires Wifi telemetry adapter)
  * Any MacBook or Ubuntu Linux laptop (can be the development computer)
  * Samsung Note 4 or equivalent (any recent Android tablet or phone with a large enough screen to run *QGroundControl* effectively).
* Safety glasses
* For multicopters - tether for more risky tests

## Vehicle Configuration

> **Tip** *QGroundControl* for a **desktop OS** is required for vehicle configuration. You should use (and regularly update) the daily build in order to take advantage of the latest features in PX4.

To configure the vehicle:

1. Download the [QGroundControl Daily Build](https://docs.qgroundcontrol.com/en/releases/daily_builds.html) for your development platform.
2. [Basic Configuration](https://docs.px4.io/en/config/) (PX4 User Guide) explains how to to perform basic configuration. 
3. [Parameter Configuration](https://docs.px4.io/en/advanced_config/parameters.html) (PX4 User Guide) explains how you can find and modify individual parameters.