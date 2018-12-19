# Toolchain Windows Cygwin

Esta toolchain es portable, fácil de instalar, y fácil de usar. Es la toolchain más reciente y con mejor rendimiento para el desarrollo de la PX4 en Windows.

> **Nota** Esta es la única toolchain oficialmente soportada para la compilación de PX4 en Windows (es decir, se prueba en nuestro sistema de integración continua).

La toolchain soporta:

* Compilar/subir código de PX4 a objetivoss NuttX (controladores de la serie Pixhawk)
* Simulador JMAVSim/SITL con un rendimiento significativamente mejor que otros toolchain para Windows.
* Verificación de estilo, instalador portable, autocompletado de línea de comandos y muchas [otras características](#features).

En esta página se explica cómo descargar y usar el entorno y cómo puede ser ampliado y actualizado si es necesario (por ejemplo, para utilizar un compilador diferente).

<!-- Legacy Versions (**deprecated**):

* [PX4 Windows Cygwin Toolchain 0.4 Download](https://s3-us-west-2.amazonaws.com/px4-tools/PX4+Windows+Cygwin+Toolchain/PX4+Windows+Cygwin+Toolchain+0.4.msi) (18.09.2018)
* [PX4 Windows Cygwin Toolchain 0.3 Download](https://s3-us-west-2.amazonaws.com/px4-tools/PX4+Windows+Cygwin+Toolchain/PX4+Windows+Cygwin+Toolchain+0.3.msi) (25.07.2018)
* [PX4 Windows Cygwin Toolchain 0.2 Download](https://s3-us-west-2.amazonaws.com/px4-tools/PX4+Windows+Cygwin+Toolchain/PX4+Windows+Cygwin+Toolchain+0.2.msi) (09.05.2018)
* [PX4 Windows Cygwin Toolchain 0.1 Download](https://s3-us-west-2.amazonaws.com/px4-tools/PX4+Windows+Cygwin+Toolchain/PX4+Windows+Cygwin+Toolchain+0.1.msi) (23.02.2018)
-->

## Instrucciones de Instalación {#installation}

1. Descarga la última versión del instalador MSI ready-to-use desde [Github](https://github.com/PX4/windows-toolchain/releases) o [S3](https://s3-us-west-2.amazonaws.com/px4-tools/PX4+Windows+Cygwin+Toolchain/PX4+Windows+Cygwin+Toolchain+0.5.msi)
2. Ejecútalo, elige el lugar de instalación deseado, instálalo ![jMAVSimOnWindows](../../assets/toolchain/cygwin_toolchain_installer.PNG)
3. Marque la casilla al final de la instalación para *clonar el repositorio PX4, compilar y ejecuta la simulación con jMAVSim* (Esto simplifica el proceso para empezar).
    
    > **Nota** Si no hiciste este paso será necesario [clonar el repositorio PX4 Firmware manualmente](#getting_started).

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

El directorio de instalación (por defecto: **C:\PX4**) contiene scripts batch para iniciar las ventanas de la consola e inicar diferentes IDEs dentro del entorno de la toolchain Cygwin. La lista completa de scripts proporcionados es:

* **run-console.bat** - iniciar la consola de bash POSIX (tipo linux).
* **run-eclipse.bat** - iniciar la compilación en [eclipse portable para C++ IDE](http://www.eclipse.org/downloads/eclipse-packages/).
* **run-vscode.bat** - Inicia el [IDE de Visual Studio Code](https://code.visualstudio.com/) (esto debe ser instalado por separado) desde su directorio de instalación por defecto: `C:\Program Files\Microsoft VS Code`

> **Nota** La sección de [Configuración Manual](#manual_setup) explica por qué es necesario utilizar los scripts y cómo funciona todo.

<span></span>

> **Nota** Puede crear accesos directos de escritorio para los scripts batch para un acceso más rápido, el instalador todavía no los crea (a partir de la versión 0.2 de la toolchain).

El flujo de trabajo habitual consiste iniciar una consola de windows haciendo doble clic en el script **run-console.bat** para ejecutar manualmente comandos de terminal. Los desarrolladores que prefieran un IDE puede iniciarlo con el script correspondiente **run-XXX.bat** para editar código/ejecutar compilados.

### Windows & Casos especiales en Git

#### Finales de linea Windows CR+LF vs Unix LF

Recomendamos forzar los finales de estilo LF Unix para cada repositorio en lo que se está trabajando usando esta toolchain (y usa un editor que los mantenga al guardar los cambios - por ejemplo, Eclipse o VS Code). Compilation of source files also works with CR+LF endings checked out locally, but there are cases in Cygwin (e.g. execution of shell scripts) that require Unix line endings ( otherwise you get errors like `$'\r': Command not found.`). Por suerte git puede hacer esto por usted al ejecutar los dos comandos en el directorio raíz de tu repo:

    git config core.autocrlf false
    git config core.eol lf
    

Si trabaja con este toolchain en múltiples repositorios también puede establecer estas dos configuraciones a nivel global para su máquina:

    git config --global ...
    

No es recomendable porque puede afectar a cualquier otro uso de git (sin relación) en su equipo Windows.

#### Bit de permisos de ejecución Unix

Under Unix there's a flag in the permissions of each file that tells the OS whether or not the file is allowed to be executed. *git* under Cygwin supports and cares about that bit (even though the Windows NTFS file system does not use it). This often results in *git* finding "false-positive" differences in permissions. The resulting diff might look like this:

    diff --git ...
    old mode 100644
    new mode 100755
    

We recommend globally disabling the permission check on Windows to avoid the problem:

    git config --global core.fileMode false # disable execution bit check globally for the machine
    

For existing repositories that have this problem caused by a local configuration, additionally:

    git config --unset core.filemode # remove the local option for this repository to apply the global one
    git submodule foreach --recursive git config --unset core.filemode # remove the local option for all submodules
    

## Additional Information

### Features / Issues {#features}

The following features are known to work (version 2.0):

* Building and running SITL with jMAVSim with significantly better performance than a VM (it generates a native windows binary **px4.exe**).
* Building and uploading NuttX builds (e.g.: px4_fmu-v2 and px4_fmu-v4)
* Style check with *astyle* (supports the command: `make format`)
* Command line auto completion
* Non-invasive installer! The installer does NOT affect your system and global path (it only modifies the selected installation directory e.g. **C:\PX4** and uses a temporary local path).
* The installer supports updating to a new version keeping your personal changes inside the toolchain folder

Omissions:

* Simulation: Gazebo and ROS are not supported
* Only NuttX and JMAVSim/SITL builds are supported.
* [Known problems / Report your issue](https://github.com/orgs/PX4/projects/6)

### Shell Script Installation {#script_setup}

You can also install the environment using shell scripts in the Github project.

1. Make sure you have [Git for Windows](https://git-scm.com/download/win) installed.
2. Clone the repository https://github.com/PX4/windows-toolchain to the location you want to install the toolchain. Default location and naming is achieved by opening the `Git Bash` and executing:

    cd /c/
    git clone https://github.com/PX4/windows-toolchain PX4
    

1. If you want to install all components navigate to the freshly cloned folder and double click on the script `install-all-components.bat` located in the folder `toolchain`. If you only need certain components and want to safe Internet traffic and or disk space you can navigate to the different component folders like e.g. `toolchain\cygwin64` and click on the **install-XXX.bat** scripts to only fetch something specific.
2. Continue with [Getting Started](#getting_started) (or [Usage Instructions](#usage_instructions)) 

### Manual Installation (for Toolchain Developers) {#manual_setup}

This section describes how to setup the Cygwin toolchain manually yourself while pointing to the corresponding scripts from the script based installation repo. The result should be the same as using the scripts or MSI installer.

> **Note** The toolchain gets maintained and hence these instructions might not cover every detail of all the future changes.

1. Create the *folders*: **C:\PX4**, **C:\PX4\toolchain** and **C:\PX4\home**
2. Download the *Cygwin installer* file [setup-x86_64.exe](https://cygwin.com/setup-x86_64.exe) from the [official Cygwin website](https://cygwin.com/install.html)
3. Run the downloaded setup file
4. In the wizard choose to install into the folder: **C:\PX4\toolchain\cygwin64**
5. Select to install the default Cygwin base and the newest available version of the following additional packages:

* **Category:Packagename**
* Devel:cmake (3.3.2 gives no deprecated warnings, 3.6.2 works but has the warnings)
* Devel:gcc-g++
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

1. Write up or copy the **batch scripts** [`run-console.bat`](https://github.com/MaEtUgR/PX4Toolchain/blob/master/run-console.bat) and [`setup-environment-variables.bat`](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/setup-environment-variables.bat).
    
    The reason to start all the development tools through the prepared batch scripts is they preconfigure the starting program to use the local, portable Cygwin environment inside the toolchain's folder. This is done by always first calling the script [**setup-environment-variables.bat**](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/setup-environment-variables.bat) and the desired application like the console after that.
    
    The script [`setup-environment-variables.bat`](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/setup-environment-variables.bat) locally sets environmental variables for the workspace root directory `PX4_DIR`, all binary locations `PATH`, and the home directory of the unix environment `HOME`.

2. Add necessary **python packages** to your setup by opening the Cygwin toolchain console (double clicking **run-console.bat**) and executing
    
        pip2 install toml
        pip2 install pyserial
        pip2 install pyulog
        
    
    > **Note** That's what [cygwin64/install-cygwin-python-packages.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/cygwin64/install-cygwin-python-packages.bat) does.

3. Download the [**ARM GCC compiler**](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads) as zip archive of the binaries for Windows and unpack the content to the folder `C:\PX4\toolchain\gcc-arm`.
    
    > **Note** This is what the toolchain does in: [gcc-arm/install-gcc-arm.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/gcc-arm/install-gcc-arm.bat).

4. Install the JDK:
    
    * Download the [**Java Development Kit Installer**](http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html).
    * Because sadly there is no portable archive containing the binaries directly you have to install it.
    * Find the binaries and move/copy them to **C:\PX4\toolchain\jdk**.
    * You can uninstall the Kit from your Windows system again, we only needed the binaries for the toolchain.
    
    > **Note** This is what the toolchain does in: [jdk/install-jdk.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/jdk/install-jdk.bat).

5. Download [**Apache Ant**](https://ant.apache.org/bindownload.cgi) as zip archive of the binaries for Windows and unpack the content to the folder `C:\PX4\toolchain\apache-ant`.
    
    > **Tip** Make sure you don't have an additional folder layer from the folder which is inside the downloaded archive.
    
    <span></span>
    
    > **Note** This is what the toolchain does in: [apache-ant/install-apache-ant.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/apache-ant/install-apache-ant.bat).

6. Download, build and add *genromfs* to the path:
    
    * Clone the source code to the folder **C:\PX4\toolchain\genromfs\genromfs-src** with 
            cd /c/toolchain/genromfs
            git clone https://github.com/chexum/genromfs.git genromfs-src

* Compile it with: 
    
        cd genromfs-src
         make all
    
    * Copy the resulting binary **genromfs.exe** one folder level out to: **C:\PX4\toolchain\genromfs**
    
    > **Note** This is what the toolchain does in: [genromfs/install-genromfs.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/genromfs/install-genromfs.bat).

1. Make sure all the binary folders of all the installed components are correctly listed in the `PATH` variable configured by [**setup-environment-variables.bat**](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/setup-environment-variables.bat).