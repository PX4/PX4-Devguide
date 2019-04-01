# Entorno de desarrollo en Linux

Linux le permite compilar para [todos los posibles objetivos de PX4](../setup/dev_env.md#supported-targets) (hardware basado en NuttX, hardware Qualcomm Snapdragon vuelo, hardware basado en Linux, simulación, ROS).

> **Tip** [Ubuntu Linux LTS](https://wiki.ubuntu.com/LTS) 16.04 is the tested/supported Linux distribution for most development. Ubuntu 18.04 LTS with ROS Melodic is used for [ROS development](#ros). Instructions are also provided for [CentOS](../setup/dev_env_linux_centos.md) and [Arch Linux](../setup/dev_env_linux_arch.md).

Las siguientes instrucciones explican cómo configurar un entorno de desarrollo en Ubuntu LTS mediante los convenientes scripts de bash. Las instrucciones para *instalar manualmente* estos y los objetivos adicionales pueden encontrarse en [Ubuntu/Debian Linux](../setup/dev_env_linux_ubuntu.md).

## Toolchain de desarrollo

Las instrucciones siguientes muestran cómo utilizar nuestra [bash scripts](../setup/dev_env_linux_ubuntu.md#convenience-bash-scripts) para configurar la toolchain de desarrollador en Ubuntu LTS. Todos los scripts instalan *Qt Creator IDE*, [Ninja Build System](https://ninja-build.org/), [dependencias comunes](../setup/dev_env_linux_ubuntu.md#common-dependencies), [FastRTPS](../setup/dev_env_linux_ubuntu.md#fastrtps-installation) y también el código fuente de la PX4 en tu ordenador (**~/src/Firmware**).

> **Tip** The scripts have been tested on clean Ubuntu LTS 16.04 and Ubuntu LTS 18.04 installations. *Podrían* no funcionar como se esperaba si se ha instalado sobre un sistema existente o en otra versión de Ubuntu. Si tiene problemas siga las [instrucciones de instalación manual](../setup/dev_env_linux_ubuntu.md).

First make the user a member of the group "dialout":

1. En la terminal escriba: 
        sh
        sudo usermod -a -G dialout $USER

2. Cierre sesión y entre de nuevo (el cambio se realiza sólo después de un nuevo inicio de sesión).

Siga las instrucciones para su objetivo de desarrollo en las secciones a continuación.

### Pixhawk/NuttX (y jMAVSim)

Para instalar la toolchain de desarrollo:

1. Download <a href="https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/ubuntu_sim_nuttx.sh" target="_blank" download>ubuntu_sim_nuttx.sh</a>.
2. Ejecutar el script en un shell de bash: 
        bash
        source ubuntu_sim_nuttx.sh Podría necesitar reconocer algunas indicaciones de como avanza el script.

3. Reinicie el equipo al finalizar.

### Snapdragon Flight

Las instrucciones de configuración para Snapdragon Flight se proporcionan en la *Guía del usuario de PX4*:

* [Entorno de desarrollo](https://docs.px4.io/en/flight_controller/snapdragon_flight_dev_environment_installation.html)
* [Instalación del software](https://docs.px4.io/en/flight_controller/snapdragon_flight_software_installation.html)
* [Configuración](https://docs.px4.io/en/flight_controller/snapdragon_flight_configuration.html)

### Raspberry Pi

Para instalar la toolchain de desarrollo:

1. Download <a href="https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/ubuntu_sim_common_deps.sh" target="_blank" download>ubuntu_sim_common_deps.sh</a> (this contains the jMAVSim simulator and common toolchain dependencies).
2. Ejecutar el script en un shell de bash: 
        bash
        source ubuntu_sim_common_deps.sh Podría necesitar reconocer algunas indicaciones de como avanza el script.

3. Siga las instrucciones de instalación en [Ubuntu/Debian Linux](../setup/dev_env_linux_ubuntu.md) para [Raspberry Pi](../setup/dev_env_linux_ubuntu.md#raspberry-pi-hardware).

### Parrot Bepop

Siga las instrucciones (manuales) aquí: [Ubuntu/Debian Linux > Parrot Bebop](../setup/dev_env_linux_ubuntu.md#raspberry-pi-hardware).

### Simulación jMAVSim/Gazebo

To install the Gazebo9 and jMAVSim simulators:

1. Download <a href="https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/ubuntu_sim.sh" target="_blank" download>ubuntu_sim.sh</a>.
2. Ejecutar el script en un shell de bash: 
        bash
        source ubuntu_sim.sh Podría necesitar reconocer algunas indicaciones de como avanza el script.

> **Tip** If you just need jMAVSim, instead download and run <a href="https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/ubuntu_sim_common_deps.sh" target="_blank" download>ubuntu_sim_common_deps.sh</a>.

<span><span></p> 

<blockquote>
  <p>
    <strong>Nota</strong> PX4 funciona con Gazebo 7, 8, and 9. The script installs Gazebo 9.
  </p>
</blockquote>

<h3 id="ros">
  Gazebo with ROS Melodic
</h3>

<blockquote>
  <p>
    <strong>Note</strong> PX4 is tested with ROS Melodic on Ubuntu 18.04 LTS. ROS Melodic does not work on Ubuntu 16.04.
  </p>
</blockquote>

<p>
  To install the development toolchain:
</p>

<ol start="1">
  <li>
    Download <a href="https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/ubuntu_sim_ros_melodic.sh" target="_blank" download>ubuntu_sim_ros_melodic.sh</a>.
  </li>
  
  <li>
    Run the script in a bash shell: <pre><code>bash
source ubuntu_sim_ros_gazebo.sh</code></pre> You may need to acknowledge some prompts as the script progresses.
  </li>
</ol>

<p>
  Note:
</p>

<ul>
  <li>
    ROS Melodic is installed with Gazebo9 by default.
  </li>
  <li>
    Your catkin (ROS build system) workspace is created at <strong>~/catkin_ws/</strong>.
  </li>
</ul>

<h2>
  Additional Tools
</h2>

<p>
  After setting up the build/simulation toolchain, see <a href="../setup/generic_dev_tools.md">Additional Tools</a> for information about other useful tools.
</p>

<h2>
  Next Steps
</h2>

<p>
  Once you have finished setting up the environment, continue to the <a href="../setup/building_px4.md">build instructions</a>.
</p>