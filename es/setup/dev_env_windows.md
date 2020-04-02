# Instrucciones de Instalación en Windows

Para desarrollar para PX4 en Windows, siga las instrucciones en [Windows Cygwin Toolchain](../setup/dev_env_windows_cygwin.md).

> **Nota** La *Cygwin toolchain* soporta compilación para objetivos NuttX/Pixhawk y simulador jMAVSim. Si quiere compilar para [otros objetivos](/setup/dev_env.md#supported-targets), considere contar con un sistema dual boot con [Ubuntu Linux](http://ubuntu.com).

## Herramientas adicionales

Después de configurar la toolchain de compilación/simulación, consulte [Herramientas adicionales](../setup/generic_dev_tools.md) para obtener información sobre otras herramientas de "desarrollo general" útiles.

## Siguientes Pasos

Una vez que haya terminado de configurar el entorno, continúe a [Compilando el código](../setup/building_px4.md).

## Otras Toolchains en Windows

Hay un número de otras soluciones legacy/alternativas que podrían ser de interés para algunos desarrolladores. Se agrega una comparación a continuación.

> **Nota** [Cygwin Toolchain](../setup/dev_env_windows_cygwin.md) es el único que cuenta con soporte por el equipo de desarrollo de PX4. Es regularmente probado como parte de nuestro sistema de integración continua y es conocido por tener mejor funcionamiento que otras alternativas.

|                          | [Cygwin Toolchain](../setup/dev_env_windows_cygwin.md) **(Soportado)** | [Virtual Machine Toolchain](../setup/dev_env_windows_vm.md) | [Bash on Windows Toolchain](../setup/dev_env_windows_bash_on_win.md) |
| ------------------------ | ---------------------------------------------------------------------- | ----------------------------------------------------------- | -------------------------------------------------------------------- |
| Instalación              | MSI installer o Script                                                 | Script                                                      | Script                                                               |
| Ejecución nativa binaria | sí                                                                     | no                                                          | no                                                                   |
| Rendimiento              | ++                                                                     | --                                                          | -                                                                    |
| Objetivos ARM            | ++ (rápido)                                                            | + (VM USB)                                                  | +                                                                    |
| Simulación jMAVSim       | ++                                                                     | +                                                           | +                                                                    |
| Simulación gazebo        | - (no aún)                                                             | + (lento)                                                   | + (lento)                                                            |
| Soporte                  | +                                                                      | ++ (Linux)                                                  | +/-                                                                  |
| Comentarios              |                                                                        |                                                             |                                                                      |

- Nuevo en 2018
- Slim setup
- Portable

|

- Completo en funcionalidades Linux
- Uso intenso en CPU & RAM
- Uso intenso de espacio en disco

|

- Simulación UI es un "hack".
- Sólo Windows 10
- Esencialmente una VM

|