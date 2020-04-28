# Toolchain Windows Cygwin

Esta toolchain es portable, fácil de instalar, y fácil de usar. Es la toolchain más reciente y con mejor rendimiento para el desarrollo de la PX4 en Windows.

> **Nota** Esta es la única toolchain oficialmente soportada para la compilación de PX4 en Windows (es decir, se prueba en nuestro sistema de integración continua).

La toolchain soporta:

* Compilar/subir código de PX4 a objetivoss NuttX (controladores de la serie Pixhawk)
* Simulador JMAVSim/SITL con un rendimiento significativamente mejor que otros toolchain para Windows.
* Verificación de estilo, instalador portable, autocompletado de línea de comandos y muchas [otras características](#features).

En esta página se explica cómo descargar y usar el entorno y cómo puede ser ampliado y actualizado si es necesario (por ejemplo, para utilizar un compilador diferente).

## Instrucciones de Instalación {#installation}

1. Download the latest version of the ready-to-use MSI installer from: [Github releases](https://github.com/PX4/windows-toolchain/releases) or [Amazon S3](https://s3-us-west-2.amazonaws.com/px4-tools/PX4+Windows+Cygwin+Toolchain/PX4+Windows+Cygwin+Toolchain+0.9.msi) (fast download).
2. Run it, choose your desired installation location, let it install: ![jMAVSimOnWindows](../../assets/toolchain/cygwin_toolchain_installer.PNG)
3. Marque la casilla al final de la instalación para *clonar el repositorio PX4, compilar y ejecuta la simulación con jMAVSim* (Esto simplifica el proceso para empezar).
    
    > **Note** If you missed this step you will need to [clone the PX4 Firmware repository manually](#getting_started).

## Primeros pasos {#getting_started}

La toolchain utiliza una ventana de consola especialmente configurada (iniciada ejecutando el script **run-console.bat**) desde el que se puede llamar a los comandos de compilación de PX4:

1. Busque el directorio de instalación de la toolchain (por defecto **C:\PX4**)
2. Ejecute **run-console.bat** (doble click) para iniciar la consola de bash de Cygwin
3. Clone el repositorio de la PX4 Firmware desde la consola:
    
    > **Nota** ¡La clonación solo necesita hacerse una vez! Omita este paso si usted marcó la opción del instalador para *clonar el repositorio PX4, compilar y ejecutar la simulación con jMAVSim*.
    
    ```bash
    # Clona el repositorio del Firmware PX4 en la carpeta de inicio & carga submódulos en paralelo
    git clone--recursive-j8 https://github.com/PX4/Firmware.git
    ```
    
    Ahora puede usar el repositorio de Firmware de consola para compilar PX4.

4. Por ejemplo, para ejecutar JMAVSim:
    
    ```bash
    # Navegue al repo Firmware
    cd Firmware
    # Compile y ejecute la simulación SITL con jMAVSim para probar la configuración
    make px4_sitl jmavsim
    ```
    
    La consola mostrará:
    
    ![jMAVSimOnWindows](../../assets/simulation/jmavsim_windows_cygwin.PNG)

Continúe en las [instrucciones que detallan cómo compilar PX4](../setup/building_px4.md)(o consulte la siguiente sección para obtener instrucciones de uso más general).

## Instrucciones de uso {#usage_instructions}

The installation directory (default: **C:\PX4**) contains a batch script for launching the PX4 SITL (linux like) bash console: **run-console.bat**

> **Tip** The [Manual Setup](#manual_setup) section explains why you need to use the script and how it all works.

The ordinary workflow consists of starting a console window by double clicking on the **run-console.bat** script to manually run terminal commands.

### File Monitoring Tools vs Toolchain Speed

Antivirus and other background file monitoring tools can significantly slow down both installation of the toolchain and PX4 build times.

You may wish to halt them temporarily during builds (at your own risk).

### Windows & Git Special Cases

#### Finales de linea Windows CR+LF vs Unix LF

We recommend that you force Unix style LF endings for every repository you're working with using this toolchain (and use an editor which preserves them when saving your changes - e.g. Eclipse or VS Code). Compilation of source files also works with CR+LF endings checked out locally, but there are cases in Cygwin (e.g. execution of shell scripts) that require Unix line endings (otherwise you get errors like `$'\r': Command not found.`). Luckily git can do this for you when you execute the two commands in the root directory of your repo:

    git config core.autocrlf false
    git config core.eol lf
    

If you work with this toolchain on multiple repositories you can also set these two configurations globally for your machine:

    git config --global ...
    

This is not recommended because it may affect any other (unrelated) git use on your Windows machine.

#### Bit de permisos de ejecución Unix

Under Unix there's a flag in the permissions of each file that tells the OS whether or not the file is allowed to be executed. *git* under Cygwin supports and cares about that bit (even though the Windows NTFS file system does not use it). This often results in *git* finding "false-positive" differences in permissions. The resulting diff might look like this:

    diff --git ...
    old mode 100644
    new mode 100755
    

We recommend globally disabling the permission check on Windows to avoid the problem:

    git config --global core.fileMode false # deshabilita el bit de comprobación de ejecución globalmente para la máquina
    

For existing repositories that have this problem caused by a local configuration, additionally:

    git config --unset core.filemode # elimina la opción local para este repositorio de aplicarlo globalmente
    git submodule foreach --recursive git config --unset core.filemode # elimina la opción local para todos los submódulos
    

## Información adicional

### Features / Issues {#features}

The following features are known to work (version 2.0):

* Building and running SITL with jMAVSim with significantly better performance than a VM (it generates a native windows binary **px4.exe**).
* Building and uploading NuttX builds (e.g.: px4_fmu-v2 and px4_fmu-v4)
* Style check with *astyle* (supports the command: `make format`)
* Command line auto completion
* Non-invasive installer! The installer does NOT affect your system and global path (it only modifies the selected installation directory e.g. **C:\PX4** and uses a temporary local path).
* The installer supports updating to a new version keeping your personal changes inside the toolchain folder

Omissions:

* Simulation: Gazebo and ROS are not supported.
* Only NuttX and JMAVSim/SITL builds are supported.
* [Known problems](https://github.com/orgs/PX4/projects/6) (Also use to report issues).

### Shell Script Installation {#script_setup}

You can also install the environment using shell scripts in the Github project.

1. Asegúrese de que tener [Git para Windows](https://git-scm.com/download/win) instalado.
2. Clone el repositorio https://github.com/PX4/windows-toolchain en la ubicación que desea instalar el toolchain. La nomenclatura y ubicación predeterminada se logra abriendo `Git Bash` y ejecutando:

    cd /c/
    git clone https://github.com/PX4/windows-toolchain PX4
    

1. Si desea instalar todos los componentes navegue a la carpeta recién clonada y haga doble clic en el script `install-all-components.bat` ubicado en la carpeta `toolchain`. Si solo necesitas ciertos componentes y quieres ahorrar tráfico de internet o espacio en disco puedes navegar a la carpeta de componente, como por ejemplo `toolchain\cygwin64`, y hacer click en el script **install-XXX.bat** para solo tener encuenta ese específico.
2. Continúe con [Introducción](#getting_started) (o [Instrucciones de Uso](#usage_instructions)) 

### Manual Installation (for Toolchain Developers) {#manual_setup}

This section describes how to setup the Cygwin toolchain manually yourself while pointing to the corresponding scripts from the script based installation repo. The result should be the same as using the scripts or MSI installer.

> **Note** The toolchain gets maintained and hence these instructions might not cover every detail of all the future changes.

1. Cree las *carpetas*: **C:\PX4**, **C:\PX4\toolchain** y **C:\PX4\home**
2. Descargar el archivo *instalador Cygwin* [setup-x86_64.exe](https://cygwin.com/setup-x86_64.exe) desde la [Web oficial de Cygwin](https://cygwin.com/install.html)
3. Ejecute el archivo de instalación descargado
4. En el asistente elija instalar en la carpeta: **C:\PX4\toolchain\cygwin64**
5. Seleccione instalar el Cygwin base por defecto y la última versión disponible de los siguientes paquetes adicionales:

* **Category:Packagename**
* Devel:cmake (3.3.2 gives no deprecated warnings, 3.6.2 works but has the warnings)
* Devel:gcc-g++
* Devel:gdb
* Devel:git
* Devel:make
* Devel:ninja
* Devel:patch
* Editors:xxd
* Editors:nano (unless you're the vim pro)
* Python:python2
* Python:python2-pip
* Python:python2-numpy
* Python:python2-jinja2
* Python:python2-pyyaml
* Python:python2-cerberus
* Archive:unzip
* Utils:astyle
* Shells:bash-completion
* Web:wget
    
    > **Note** Do not select as many packages as possible which are not on this list, there are some which conflict and break the builds.
    
    <span></span>
    
    > **Note** That's what [cygwin64/install-cygwin-px4.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/cygwin64/install-cygwin-px4.bat) does.

1. Write up or copy the **batch scripts** [`run-console.bat`](https://github.com/MaEtUgR/PX4Toolchain/blob/master/run-console.bat) and [`setup-environment.bat`](https://github.com/PX4/windows-toolchain/blob/master/toolchain/scripts/setup-environment.bat).
    
    The reason to start all the development tools through the prepared batch script is they preconfigure the starting program to use the local, portable Cygwin environment inside the toolchain's folder. This is done by always first calling the script [**setup-environment.bat**](https://github.com/PX4/windows-toolchain/blob/master/toolchain/scripts/setup-environment.bat) and the desired application like the console after that.
    
    The script [setup-environment.bat](https://github.com/PX4/windows-toolchain/blob/master/toolchain/scripts/setup-environment.bat) locally sets environmental variables for the workspace root directory `PX4_DIR`, all binary locations `PATH`, and the home directory of the unix environment `HOME`.

2. Añade los **paquetes de python** necesarios para la configuración abriendo la consola de la toolchain de Cygwin (doble clic en **run-console.bat**) y ejecutandolo
    
        pip2 install toml
        pip2 install pyserial
        pip2 install pyulog
        
    
    > **Nota** Es lo que hace [cygwin64/install-cygwin-python-packages.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/cygwin64/install-cygwin-python-packages.bat).

3. Descargar el [**compilador ARM GCC**](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads) como archivo zip de los binarios para Windows y descomprime el contenido a la carpeta `C:\PX4\toolchain\gcc-arm`.
    
    > **Nota** Esto es lo que hace la toolchain hace en: [gcc-arm/install-gcc-arm.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/gcc-arm/install-gcc-arm.bat).

4. Instala el JDK:
    
    * Descargar el [**Instalador de Kit de desarrollo Java**](http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html).
    * Porque lamentablemente no hay ningún archivo portable que contenga los binarios directamente, tienes que instalarlos.
    * Encuentra los binarios y muevelos/copialos a **C:\PX4\toolchain\jdk**.
    * Puede desinstalar el Kit de su sistema Windows otra vez, sólo necesitábamos los binarios para la toolchain.
    
    > **Nota** Esto es lo que hace la toolchain: [jdk/install-jdk.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/jdk/install-jdk.bat).

5. Descarga [**Apache Ant**](https://ant.apache.org/bindownload.cgi) como archivo zip de los binarios para Windows y descomprime el contenido a la carpeta `C:\PX4\toolchain\apache-ant`.
    
    > **Nota** Asegúrate de que no tienes una capa de carpeta adicional desde la carpeta donde esta el archivo descargado.
    
    <span></span>
    
    > **Nota** Esto es lo que hace la toolchain: [apache-ant/install-apache-ant.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/apache-ant/install-apache-ant.bat).

6. Descarga, compila y añade *genromfs* a la ruta:
    
    * Clona el código fuente en la carpeta **C:\PX4\toolchain\genromfs\genromfs-src** con 
            cd /c/toolchain/genromfs
            git clone https://github.com/chexum/genromfs.git genromfs-src

* Compile it with: 
    
        cd genromfs-src
         make all
    
    * Copy the resulting binary **genromfs.exe** one folder level out to: **C:\PX4\toolchain\genromfs**
    
    > **Note** This is what the toolchain does in: [genromfs/install-genromfs.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/genromfs/install-genromfs.bat).

1. Make sure all the binary folders of all the installed components are correctly listed in the `PATH` variable configured by [**setup-environment.bat**](https://github.com/PX4/windows-toolchain/blob/master/toolchain/scripts/setup-environment.bat).