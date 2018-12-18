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
    build-essential genromfs ninja-build exiftool vim-common -y
# Paquetes de python requeridos
sudo apt-get install python-argparse \
    python-empy python-toml python-numpy python-yaml \
    python-dev python-pip -y
sudo -H pip install --upgrade pip 
sudo -H pip install pandas jinja2 pyserial cerberus
```

Podrías también desear instalar [pyulog](https://github.com/PX4/pyulog#pyulog). Este es un útil paquete de python que contiene scripts para analizar archivos *ULog* y mostrar su contenido.

    # herramientas optionales de python
    sudo -H pip install pyulog
    

<!-- import docs ninja build system --> {% include "_ninja_build_system.txt" %}

## Instalación de FastRTPS

[eProsima Fast RTPS](http://eprosima-fast-rtps.readthedocs.io/en/latest/) is a C++ implementation of the RTPS (Real Time Publish Subscribe) protocol. FastRTPS is used, via the [RTPS/ROS2 Interface: PX4-FastRTPS Bridge](../middleware/micrortps.md), to allow PX4 uORB topics to be shared with offboard components.

The following instructions can be used to install the FastRTPS 1.5 binaries to your home directory.

```sh
wget http://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-rtps/eprosima-fast-rtps-1-5-0/eprosima_fastrtps-1-5-0-linux-tar-gz -O eprosima_fastrtps-1-5-0-linux.tar.gz
tar -xzf eprosima_fastrtps-1-5-0-linux.tar.gz eProsima_FastRTPS-1.5.0-Linux/
tar -xzf eprosima_fastrtps-1-5-0-linux.tar.gz requiredcomponents
tar -xzf requiredcomponents/eProsima_FastCDR-1.0.7-Linux.tar.gz
```

> **Note** In the following lines where we compile the FastCDR and FastRTPS libraries, the `make` command is issued with the `-j2` option. This option defines the number of parallel threads (or `j`obs) that are used to compile the source code. Change `-j2` to `-j<number_of_cpu_cores_in_your_system>` to speed up the compilation of the libraries.

```sh
cd eProsima_FastCDR-1.0.7-Linux; ./configure --libdir=/usr/lib; make -j2; sudo make install
cd ..
cd eProsima_FastRTPS-1.5.0-Linux; ./configure --libdir=/usr/lib; make -j2; sudo make install
cd ..
rm -rf requiredcomponents eprosima_fastrtps-1-5-0-linux.tar.gz
```

> **Note** More "generic" instructions, which additionally cover installation from source, can be found here: [Fast RTPS installation](../setup/fast-rtps-installation.md).

## Simulation Dependencies

The dependencies for the Gazebo and jMAVSim simulators listed below. You should minimally install jMAVSim to make it easy to test the installation. Additional information about these and other supported simulators is covered in: [Simulation](../simulation/README.md).

### jMAVSim

Install the dependencies for [jMAVSim Simulation](../simulation/jmavsim.md).

    # jMAVSim simulator
    sudo apt-get install ant openjdk-8-jdk openjdk-8-jre -y
    

### Gazebo

> **Note** If you're going work with ROS then follow the [ROS/Gazebo](#rosgazebo) instructions in the following section (these install Gazebo automatically, as part of the ROS installation).

Install the dependencies for [Gazebo Simulation](../simulation/gazebo.md).

    # Gazebo simulator
    sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    ## Setup keys
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    ## Update the debian database:
    sudo apt-get update -y
    ## Install Gazebo9
    sudo apt-get install gazebo9 -y
    ## For developers (who work on top of Gazebo) one extra package
    sudo apt-get install libgazebo9-dev -y
    

> **Tip** PX4 works with Gazebo 7, 8, and 9. The [installation instructions](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install) above are for installing Gazebo 9.

<!-- these dependencies left over when I separated the dependencies. These appear to both be for using Clang. MOve them down?
sudo apt-get install clang-3.5 lldb-3.5 -y
-->

### ROS/Gazebo

Install the dependencies for [ROS/Gazebo](../ros/README.md) ("Kinetic"). These include Gazebo7 (at time of writing, the default version that comes with ROS). The instructions come from the ROS Wiki [Ubuntu page](http://wiki.ros.org/kinetic/Installation/Ubuntu).

```sh
# ROS Kinetic/Gazebo
## Gazebo dependencies
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y

## ROS Gazebo: http://wiki.ros.org/kinetic/Installation/Ubuntu
## Setup keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
## For keyserver connection problems substitute hkp://pgp.mit.edu:80 or hkp://keyserver.ubuntu.com:80 above.
sudo apt-get update
## Get ROS/Gazebo
sudo apt-get install ros-kinetic-desktop-full -y
## Initialize rosdep
sudo rosdep init
rosdep update
## Setup environment variables
rossource="source /opt/ros/kinetic/setup.bash"
if grep -Fxq "$rossource" ~/.bashrc; then echo ROS setup.bash already in .bashrc;
else echo "$rossource" >> ~/.bashrc; fi
source ~/.bashrc
## Get rosinstall
sudo apt-get install python-rosinstall -y
```

Install the [MAVROS \(MAVLink on ROS\)](../ros/mavros_installation.md) package. This enables MAVLink communication between computers running ROS, MAVLink enabled autopilots, and MAVLink enabled GCS.

> **Tip** MAVROS can be installed as an ubuntu package or from source. Source is recommended for developers.

```sh
## Create catkin workspace (ROS build system)
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws

## Install dependencies
sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools -y

## Initialise wstool
wstool init ~/catkin_ws/src

## Build MAVROS
### Get source (upstream - released)
rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
### Get latest released mavlink package
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
### Setup workspace & install deps
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
```

> **Note** If you use an ubuntu-based distro and the command `rosdep install --from-paths src --ignore-src --rosdistro kinetic -y` fails, you can try to force the command to run by executing `rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --os ubuntu:xenial`

```sh
## Build!
catkin build
## Re-source environment to reflect new packages/build environment
catkin_ws_source="source ~/catkin_ws/devel/setup.bash"
if grep -Fxq "$catkin_ws_source" ~/.bashrc; then echo ROS catkin_ws setup.bash already in .bashrc;
else echo "$catkin_ws_source" >> ~/.bashrc; fi
source ~/.bashrc
```

## NuttX-based Hardware

Install the following dependencies to build for NuttX based hardware: Pixhawk, Pixfalcon, Pixracer, Pixhawk 3, Intel® Aero Ready to Fly Drone.

> **Note** Packages with specified versions should be installed with the specified package version.

```sh
sudo apt-get install python-serial openocd \
    flex bison libncurses5-dev autoconf texinfo \
    libftdi-dev libtool zlib1g-dev -y
```

Remove any old versions of the arm-none-eabi toolchain.

```sh
sudo apt-get remove gcc-arm-none-eabi gdb-arm-none-eabi binutils-arm-none-eabi gcc-arm-embedded
sudo add-apt-repository --remove ppa:team-gcc-arm-embedded/ppa
```

<!-- import GCC toolchain common documentation --> {% include "_gcc_toolchain_installation.txt" %}

## Snapdragon Flight

Setup instructions for Snapdragon Flight are provided in the *PX4 User Guide*:

* [Development Environment](https://docs.px4.io/en/flight_controller/snapdragon_flight_dev_environment_installation.html)
* [Software Installation](https://docs.px4.io/en/flight_controller/snapdragon_flight_software_installation.html)
* [Configuration](https://docs.px4.io/en/flight_controller/snapdragon_flight_configuration.html)

## Raspberry Pi Hardware

Developers working on Raspberry Pi hardware need to download a ARMv7 cross-compiler, either GCC or clang. The current recommended toolchain for raspbian can be cloned from `https://github.com/raspberrypi/tools.git` (at time of writing 4.9.3). The `PATH` environmental variable should include the path to the gcc cross-compiler collection of tools (e.g. gcc, g++, strip) prefixed with `arm-linux-gnueabihf-`.

```sh
git clone https://github.com/raspberrypi/tools.git ${HOME}/rpi-tools

# test compiler
$HOME/rpi-tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-gcc -v

# permanently update PATH variable by modifying ~/.profile
echo 'export PATH=$PATH:$HOME/rpi-tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin' >> ~/.profile

# update PATH variable only for this session
export PATH=$PATH:$HOME/rpi-tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin
```

### clang

In order to use clang, you also need GCC.

Download clang for your specific distribution from [LLVM Download page](http://releases.llvm.org/download.html) and unpack it. Assuming that you've unpacked clang to `CLANG_DIR`, and `clang` binary is available in `CLANG_DIR/bin`, and you have the GCC cross-compiler in `GCC_DIR`, you will need to setup the symlinks for clang in the `GCC_DIR` bin dir, and add `GCC_DIR/bin` to `PATH`.

Example below for building PX4 firmware out of tree, using CMake.

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

### Native Builds

Additional developer information for using PX4 on Raspberry Pi (including building PX4 natively) can be found here: [Raspberry Pi 2/3 Navio2 Autopilot](https://docs.px4.io/en/flight_controller/raspberry_pi_navio2.html).

## Parrot Bebop

Developers working with the Parrot Bebop should install the RPi Linux Toolchain. Follow the description under [Raspberry Pi hardware](#raspberry-pi-hardware).

Next, install ADB.

```sh
sudo apt-get install android-tools-adb -y
```

## Additional Tools

After setting up the build/simulation toolchain, see [Additional Tools](../setup/generic_dev_tools.md) for information about other useful tools.

## Next Steps

Once you have finished setting up the environment, continue to the [build instructions](../setup/building_px4.md).