# Toolchain Bash on Windows

> **Nota** [Cygwin Toolchain](../setup/dev_env_windows_cygwin.md) es el (único) toolchain que cuenta con soporte para desarrollo en Windows.

Los usuarios de Windows también pueden instalar un entorno de desarrollo PX4 Ubuntu Linux *ligeramente modificados* en [Bash on Windows](https://github.com/Microsoft/BashOnWindows) y utilizarlo para:

* Compilar el firmware para objetivos NuttX/Pixhawk. 
* Ejecutar la simulación PX4 JMAVSim (usando una aplicación alojada en Windows X-Windows para mostrar la interfaz de usuario)

> **Nota** Este mecanismo sólo funciona en Windows 10. Esencialmente funciona el toolchain en una máquina virtual y es relativamente lento en comparación con otras soluciones.

### Configura el entorno

La forma más fácil de configurar el entorno es utilizar el script **<a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/windows_bash_nuttx.sh" target="_blank" download>windows_bash_nuttx.sh</a>** (los detalles del script están [más abajo](#build_script_details)).

Para configurar el entorno de desarrollo:

1. Instala [Bash on Windows](https://github.com/Microsoft/BashOnWindows).
2. Abre el shell de bash. 
3. Descarga el **windows_bash_nuttx.sh**: ```wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/windows_bash_nuttx.sh```
4. Ejecuta el script usando el siguiente comando (reconociendo cualquier mensaje que sea necesario): 
        sh
        source windows_bash_nuttx.sh

### Compila el Firmware

Para compilar el firmware (por ejemplo para px4_fmu-v4):

1. Introduzca los siguientes comandos en el shell de bash:
    
        cd ~/src/Firmware
        make px4_fmu-v4_default
        
    
    Una vez se haya completado encontrarás el firmware aquí: `Firmware/build/px4_fmu-v4_default/px4_fmu-v4_default.px4`
    
    > **Nota** Los comandos `make` para compilar el firmware para otras plataformas se encuentra en [Compilando el código](../setup/building_px4.md#nuttx--pixhawk-based-boards)

2. Puedes flashear el firmware custom en Windows usando *QGroundControl* o *Mission Planner* (no es posible flashear directamente desde el shell bash usando el comando `upload`).

### Simulación (jMAVSim)

Bash on Windows no incluye soporte para librerías de interfaces de usuario. Para mostrar la UI de jMAVSim primero necesitarás instalar una aplicación X-Window como [XMing](https://sourceforge.net/projects/xming/) en Windows.

Para ejecutar JMAVSim:

1. Instala y ejecuta [XMing](https://sourceforge.net/projects/xming/) en Windows.
2. Introduzca los siguientes comandos en el shell de bash: 
        sh
        export DISPLAY=:0 > 
    
    **Nota** Agrega esta linea al archivo de Ubuntu **.bashrc** si no quieres tener que ejecutarla en cada sesión.
3. Inicia PX4 y jMAVSim en el shell de bash:
    
    ```sh
    make px4_sitl jmavsim
    ```
    
    La UI de JMAVSim se mostrará en XMing tal como se muestra a continuación:
    
    ![jMAVSimOnWindows](../../assets/simulation/JMAVSim_on_Windows.PNG)

> **Advertencia** Gazebo can similarly be run within Ubuntu Bash for Windows, but too slow to be useful. To try this, follow the [ROS kinetic install guide](http://wiki.ros.org/kinetic/Installation/Ubuntu) and run Gazebo in the Bash shell as shown: 
> 
>     sh
>       export DISPLAY=:0
>       export GAZEBO_IP=127.0.0.1
>       make px4_sitl gazebo

### Build Script Details {#build_script_details}

The [windows_bash_nuttx.sh](https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/windows_bash_nuttx.sh) build script modifies the Ubuntu build instructions to remove Ubuntu-specific and UI-dependent components, including the *Qt Creator* IDE and the simulators.

In addition, it uses a [64 bit arm-none-eabi compiler](https://github.com/SolinGuo/arm-none-eabi-bash-on-win10-.git) since BashOnWindows doesn't run 32 bit ELF programs (and the default compiler from `https://launchpad.net/gcc-arm-embedded` is 32 bit).

To add this compiler to your environment manually:

1. Download the compiler: 
        sh
        wget https://github.com/SolinGuo/arm-none-eabi-bash-on-win10-/raw/master/gcc-arm-none-eabi-5_4-2017q2-20170512-linux.tar.bz2

2. Unpack it using this command line in the Bash On Windows console: 
        sh
        tar -xvf gcc-arm-none-eabi-5_4-2017q2-20170512-linux.tar.bz2 This will unpack the arm gcc cross-compiler to: ```gcc-arm-none-eabi-5_4-2017q2/bin```

3. Add the to the environment (add the line to your bash profile to make the change permanent) ```export PATH=$HOME/gcc-arm-none-eabi-5_4-2017q2/bin:\$PATH```