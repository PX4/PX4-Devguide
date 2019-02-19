# Entorno de desarrollo en Ubuntu LTS / Debian Linux

[Ubuntu Linux LTS](https://wiki.ubuntu.com/LTS) (16.04) es el SO de desarrollo Linux estándar/preferido. Le permite compilar para [todos los posibles objetivos de PX4](../setup/dev_env.md#supported-targets) (hardware basado en NuttX, hardware Qualcomm Snapdragon Flight, hardware basado en Linux, simulación, ROS).

Las siguientes instrucciones explican cómo configurar *manualmente* un entorno de desarrollo para cada uno de los objetivos compatibles.

> **Nota** Te recomendamos que uses el conveniente [scripts bash](#convenience-bash-scripts) para instalar los simuladores y/o toolchain de NuttX (esto es más fácil que escribir en las instrucciones a continuación). Siga simplemente las instrucciones adicionales para otros objetivos (por ejemplo Qualcomm Snapdragon Flight, Bebop, Raspberry Pi, etcetera)

<span></span>

> **Nota** Después de configurar la toolchain de compilación/simulación, consulte [Herramientas adicionales](../setup/generic_dev_tools.md) para obtener información sobre otras herramientas útiles.

## Scripts de Bash

Hemos creado una serie de scripts de bash que se pueden utilizar para instalar los simuladores y/o toolchain de NuttX. Todos los scripts instalan *Qt Creator IDE*, [Ninja Build System](#ninja-build-system), [dependencias comunes](#common-dependencies), [FastRTPS](#fastrtps-installation) y también descargan el código fuente de PX4 en tu ordenador (**~/src/Firmware**).

> **Nota** Los scripts han sido probados en una instalación limpia de Ubuntu LTS 16.04. *Podrían* no funcionar como se esperaba si se ha instalado sobre un sistema ya existente.

Los scripts son:

* **<a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_common_deps.sh" target="_blank" download>ubuntu_sim_common_deps.sh</a>**: [dependencias comunes](#common-dependencies), simulador de [jMAVSim](#jmavsim)
  
  * Este script contiene las dependencias comunes para todos los objetivos de compilación de PX4. Es automáticamente descargado y se ejecuta cuando se llama a cualquiera de los otros scripts.
  * Puedes ejecutar este antes de instalar las dependencias restantes para [Qualcomm Snapdragon Flight](#snapdragon-flight) o [Raspberry Pi/Parrot Bebop](#raspberry-pi-hardware).

* **<a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim.sh" target="_blank" download>ubuntu_sim.sh</a>**: **ubuntu_sim_common_deps.sh** + simulador [Gazebo8](#gazebo).

* **<a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_nuttx.sh" target="_blank" download>ubuntu_sim_nuttx.sh</a>**: **ubuntu_sim.sh** + herramientas de NuttX. 
  * *Requiere reiniciar el ordenador completamente.*
* **<a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_gazebo.sh" target="_blank" download>ubuntu_sim_ros_gazebo.sh</a>**: **ubuntu_sim_common_deps.sh** + [ROS/Gazebo y MAVROS](#rosgazebo). 
  * ROS Kinetic es instalado con Gazebo7 por defecto (hemos elegido usarlo antes que Gazebo 8 para simplificar el desarrollo en ROS).
  * Catkin (ROS build system) workspace es creado en **~/catkin_ws/**.

### Cómo usar los scripts

Para usar los scripts:

1. Hacer al usuario miembro del grupo "dialout" (esto solo hay que hacerlo una vez): 
  1. Abrir una terminal e introducir el siguiente comando: 
        sh
          sudo usermod -a -G dialout $USER
  
  2. Cierra sesión y accede de nuevo (el cambio sólo es efectivo después de un nuevo inicio de sesión).
2. Descarga el script deseado
3. Ejecuta el script en un shell bash (por ejemplo para ejecutar **ubuntu_sim.sh**): 
      bash
       source ubuntu_sim.sh Reconozca cualquier mensaje a medida que progresen los scripts.

## Configuración de permisos

> **Precaución** Nunca arreglar problemas de permiso usando `sudo`. Creará mas problemas de permiso en el proceso y requerirá reinstalación del sistema para arreglarlos.

El usuario necesita ser parte del grupo "dialout":

```sh
sudo usermod -a -G dialout $USER
```

Cierra sesión y accede de nuevo (el cambio sólo es efectivo después de un nuevo inicio de sesión).

## Eliminar el modemmanager

Ubuntu incluye un serial modem manager que interfiere agresivamente con cualquier uso de un puerto serie \(o serial USB\). Puede ser eliminado/desinstalado sin efectos secundarios:

```sh
sudo apt-get remove modemmanager
```

## Dependencias comunes

Actualiza la lista de paquetes e instala las siguientes dependencias para todos los objetivos de compilación de PX4.

```sh
sudo apt-get update -y
sudo apt-get install git zip qtcreator cmake \
    build-essential genromfs ninja-build exiftool -y

# Install xxd (package depends on version)
which xxd || sudo apt install xxd -y || sudo apt-get install vim-common --no-install-recommends -y

# Required python packages
sudo apt-get install python-argparse \
    python-empy python-toml python-numpy python-yaml \
    python-dev python-pip -y
sudo -H pip install --upgrade pip 
sudo -H pip install pandas jinja2 pyserial cerberus
```

Podrías también desear instalar [pyulog](https://github.com/PX4/pyulog#pyulog). Este es un útil paquete de python que contiene scripts para analizar archivos *ULog* y mostrar su contenido.

    # herramientas optionales de python
    sudo -H pip install pyulog
    

<!-- import docs ninja build system --> {% include "_ninja_build_system.md" %}

## Instalación de FastRTPS

[eProsima Fast RTPS](http://eprosima-fast-rtps.readthedocs.io/en/latest/) es una implementación de C++ de protocolo RTPS (Real Time Publish Subscribe). FastRTPS es usado, a través de la [interfaz RTPS/ROS2: PX4-FastRTPS Bridge](../middleware/micrortps.md), para permitir a los topic uORB de PX4 ser compartidos con componentes externos.

Las siguientes instrucciones pueden ser usadas para instalar los binarios de FastRTPS 1.5 en tu directorio home.

```sh
wget http://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-rtps/eprosima-fast-rtps-1-5-0/eprosima_fastrtps-1-5-0-linux-tar-gz -O eprosima_fastrtps-1-5-0-linux.tar.gz
tar -xzf eprosima_fastrtps-1-5-0-linux.tar.gz eProsima_FastRTPS-1.5.0-Linux/
tar -xzf eprosima_fastrtps-1-5-0-linux.tar.gz requiredcomponents
tar -xzf requiredcomponents/eProsima_FastCDR-1.0.7-Linux.tar.gz
```

> **Nota** En las siguientes líneas donde compilamos las librerías FastCDR y FastRTPS, el comando `make` es ejecutado con la opción `-j2`. Esta opción define el número de hilos paralelos (o `j`obs) que son usados para compilar el código fuente. Cambia `-j2` a `-j<number_of_cpu_cores_in_your_system>` para acelerar la compilación de las librerías.

```sh
(cd eProsima_FastCDR-1.0.7-Linux && ./configure --libdir=/usr/lib && make -j2 && sudo make install)
(cd eProsima_FastRTPS-1.5.0-Linux && ./configure --libdir=/usr/lib && make -j2 && sudo make install)
rm -rf requiredcomponents eprosima_fastrtps-1-5-0-linux.tar.gz
```

> **Nota** Instrucciones más "generic", que adicionalmente cubren la instalación del código fuente, pueden ser encontradas aquí: [Instalación de Fast RTPS](../setup/fast-rtps-installation.md).

## Dependencias de simulación

Las dependencias para los simuladores Gazebo y jMAVSim son listadas abajo. Se debería instalar como mínimo jMAVSim para hacer más sencillo probar la instalación. Información adicional sobre esas y otros simuladores soportados se comenta en: [Simulación](../simulation/README.md).

### jMAVSim

Instala las dependencias para [Simulación jMAVSim](../simulation/jmavsim.md).

    # jMAVSim simulator
    sudo apt-get install ant openjdk-8-jdk openjdk-8-jre -y
    

### Gazebo

> **Nota** Si vas a trabajar con ROS entonces sigue las instrucciones de [ROS/Gazebo](#rosgazebo) en la siguiente sección (en ellas se instala Gazebo automáticamente, como parte de la instalación de ROS).

Instala las dependencias para [simulación en Gazebo](../simulation/gazebo.md).

    # Simulador Gazebo
    sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    ## Configura las claves
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    ## Actualiza la base de datos de debian:
    sudo apt-get update -y
    ## Instala Gazebo9
    sudo apt-get install gazebo9 -y
    ## Para desarrolladores (quienes trabajen directamente con Gazebo) un paquete extra
    sudo apt-get install libgazebo9-dev -y
    

> **Nota**PX4 funciona con Gazebo 7, 8, and 9. Las [instrucciones de instalación](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install) de arriba son para instalar Gazebo 9.

<!-- these dependencies left over when I separated the dependencies. These appear to both be for using Clang. MOve them down?
sudo apt-get install clang-3.5 lldb-3.5 -y
-->

### ROS/Gazebo

Instala las dependencias para [ROS/Gazebo](../ros/README.md)("Kinetic"). Éste incluye Gazebo7 (la versión por defecto que viene con ROS). Las instrucciones vienen de la Wiki de ROS sobre [Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu).

```sh
# ROS Kinetic/Gazebo
## Dependencias de Gazebo
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y

## ROS Gazebo: http://wiki.ros.org/kinetic/Installation/Ubuntu
## Configura claves
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
## Para problemas de conexión con el servidor de claves sustituye hkp://pgp.mit.edu:80 o hkp://keyserver.ubuntu.com:80 de arriba.
sudo apt-get update
## Instala ROS/Gazebo
sudo apt-get install ros-kinetic-desktop-full -y
## Inicializa rosdep
sudo rosdep init
rosdep update
## Configura variables de entorno
rossource="source /opt/ros/kinetic/setup.bash"
if grep -Fxq "$rossource" ~/.bashrc; then echo ROS setup.bash already in .bashrc;
else echo "$rossource" >> ~/.bashrc; fi
source ~/.bashrc
## Instala rosinstall
sudo apt-get install python-rosinstall -y
```

Instala el paquete [MAVROS \(MAVLink en ROS\)](../ros/mavros_installation.md). Esto habilita comunicación MAVLink entre ordenadores que ejecuten ROS, MAVLink habilita autopilotos, y MAVLink habilita GCS.

> **Nota** MAVROS puede ser instalado como un paquete de ubuntu o compilando desde el código fuente. Desde el código fuente es recomendado para desarrolladores.

```sh
## Crea workspace de catkin (ROS build system)
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws

## Instala dependencias
sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools -y

## Inicializa wstool
wstool init ~/catkin_ws/src

## Compila MAVROS
### Instala desde el código fuente (upstream - released)
rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
### Instala latest released mavlink package
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
### Configura workspace & instala dependencias
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
```

> **Nota** Si usas una distribución basada en ubuntu y el comando `rosdep install --from-paths src --ignore-src --rosdistro kinetic -y` falla, puedes intentar forzar que funcione el comando ejecutando `rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --os ubuntu:xenial`

```sh
## Compila!
catkin build
## Re-lanza el archivo de configuración para reflejar nuevos paquetes/compilaciones en el entorno de trabajo
catkin_ws_source="source ~/catkin_ws/devel/setup.bash"
if grep -Fxq "$catkin_ws_source" ~/.bashrc; then echo ROS catkin_ws setup.bash already in .bashrc;
else echo "$catkin_ws_source" >> ~/.bashrc; fi
source ~/.bashrc
```

## Hardware basado en NuttX

Instala las siguientes dependencias para compilar para hardware basado en NuttX: Pixhawk, Pixfalcon, Pixracer, Pixhawk 3, Intel® Aero Ready to Fly Drone.

> **Nota** Paquetes con versiones específicas deberían ser instalados con la versión específica del paquete.

```sh
sudo apt-get install python-serial openocd \
    flex bison libncurses5-dev autoconf texinfo \
    libftdi-dev libtool zlib1g-dev -y
```

Elimina cualquier versión antigua de la toolchain de arm-none-eabi.

```sh
sudo apt-get remove gcc-arm-none-eabi gdb-arm-none-eabi binutils-arm-none-eabi gcc-arm-embedded
sudo add-apt-repository --remove ppa:team-gcc-arm-embedded/ppa
```

<!-- import GCC toolchain common documentation --> {% include "_gcc_toolchain_installation.md" %}

## Snapdragon Flight

Las instrucciones de configuración para Snapdragon Flight se proporcionan en la *Guía del usuario de PX4*:

* [Entorno de desarrollo](https://docs.px4.io/en/flight_controller/snapdragon_flight_dev_environment_installation.html)
* [Instalación del software](https://docs.px4.io/en/flight_controller/snapdragon_flight_software_installation.html)
* [Configuración](https://docs.px4.io/en/flight_controller/snapdragon_flight_configuration.html)

## Raspberry Pi Hardware

Los desarrolladores que trabajan en hardware Raspberry Pi necesitan descargar un compilador cruzado de ARMv7, ya sea para GCC o clang. La herramienta actualmente recomendada para raspbian puede ser clonada de `https://github.com/raspberrypi/tools.git`. La variable de entorno `PATH` debería incluir la ruta a la colección de herramientas de compilador cruzado de gcc (por ejemplo gcc, g++, strip) prefijado con `arm-linux-gnueabihf-`.

```sh
git clone https://github.com/raspberrypi/tools.git ${HOME}/rpi-tools

# test compilador
$HOME/rpi-tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-gcc -v

# permanentemente actualiza la variable PATH modificando ~/.profile
echo 'export PATH=$PATH:$HOME/rpi-tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin' >> ~/.profile

# actualiza la variable PATH solo para esta sesión
export PATH=$PATH:$HOME/rpi-tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin
```

### clang

Para usar clang, también es necesario GCC.

Descarga clang para tu distribución específica desde [LLVM Download page](http://releases.llvm.org/download.html) y descomprímelo. Asumiendo que has extraído el contenido de clang a `CLANG_DIR`, y el binario de `clang` esta disponible en `CLANG_DIR/bin`, y tienes el compilador cruzado de GCC en `GCC_DIR`, necesitarás configurar los symlinks para clang en el directorio de binario `GCC_DIR`, y agregar `GCC_DIR/bin` a `PATH`.

Ejemplo a continuación de compilación del firmware de PX4, usando CMake.

```sh
ln -s <CLANG_DIR>/bin/clang <GCC_DIR>/bin/clang
ln -s <CLANG_DIR>/bin/clang++ <GCC_DIR>/bin/clang++
export PATH=<GCC_DIR>/bin:$PATH

cd <PATH-TO-PX4-SRC>
mkdir build/posix_rpi_cross_clang
cd build/posix_rpi_cross_clang
cmake \
-G"Unix Makefiles" \
-DCONFIG=posix_rpi_cross \
-DCMAKE_C_COMPILER=clang \
-DCMAKE_CXX_COMPILER=clang++ \
..
```

### Compilación Nativa

Información adicional para desarrollador para usar PX4 en Raspberry Pi (incluyendo compilación nativa de PX4) puede ser encontrado aquí: [Autopiloto Raspberry Pi 2/3 Navio2](https://docs.px4.io/en/flight_controller/raspberry_pi_navio2.html).

## Parrot Bebop

Los desarrolladores que trabajen con el Parrot Bebop deberían instalar la RPi Linux Toolchain. Siga la descripción bajo [Hardware Raspberry Pi](#raspberry-pi-hardware).

Despueés instale ADB.

```sh
sudo apt-get install android-tools-adb -y
```

## Herramientas adicionales

Después de configurar la toolchain de compilación/simulación, consulte [Herramientas adicionales](../setup/generic_dev_tools.md) para obtener información sobre otras herramientas útiles.

## Siguientes Pasos

Una vez que haya terminado de configurar el entorno, continúe a [Compilando el código](../setup/building_px4.md).