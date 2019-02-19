# Entorno de desarrollo en Mac

MacOS es una plataforma de desarrollo compatible para PX4. Las siguientes instrucciones configuran un entorno para la compilación:

* Hardware basado en NuttX (Pixhawk, etcetera.)
* Simulación en jMAVSim
* Simulación en Gazebo 8

> **Nota** Para compilar para otros objetivos véase: [Instalación de la Toolchain > Objetivos compatibles](../setup/dev_env.md#supported-targets).

## Instalación de Homebrew

La instalación de Homebrew es rápido y fácil: [instrucciones de instalación](https://brew.sh).

## Herramientas comunes

Después de instalar Homebrew, ejecuta estos comandos en la shell para instalar las herramientas comunes:

```sh
brew tap PX4/px4
brew install px4-dev
# Opcional, pero recomendadas herramientas adicionales de simulación:
brew install px4-sim
```

Si durante la instalación resulta un mensaje de error acerca de que faltan requisitos, siga las instrucciones. A su sistema le falta Java y Quartz:

```sh
brew cask install xquartz java
```

Instala pip si no lo estaba ya y usalo para instalar los paquetes requeridos:

```sh
sudo easy_install pip
sudo -H pip install pyserial empy toml numpy pandas jinja2 pyyaml
```

## Herramientas adicionales

Después de configurar la toolchain de compilación/simulación, consulte [Herramientas adicionales](../setup/generic_dev_tools.md) para obtener información sobre otras herramientas útiles.

## Siguientes Pasos

Una vez que haya terminado de configurar el entorno, continúe a [Compilando el código](../setup/building_px4.md).