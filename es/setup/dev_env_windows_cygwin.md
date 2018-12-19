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

Recomendamos forzar los finales de estilo LF Unix para cada repositorio en lo que se está trabajando usando esta toolchain (y usa un editor que los mantenga al guardar los cambios - por ejemplo, Eclipse o VS Code). También funciona con terminaciones CR+LF extraídos localmente, pero hay casos en Cygwin (por ejemplo, ejecución de scripts de shell) que requieren terminaciones de línea de Unix (de lo contrario se obtienen errores como `$' \r': comando no encontrado.`). Por suerte git puede hacer esto por usted al ejecutar los dos comandos en el directorio raíz de tu repo:

    git config core.autocrlf false
    git config core.eol lf
    

Si trabaja con este toolchain en múltiples repositorios también puede establecer estas dos configuraciones a nivel global para su máquina:

    git config --global ...
    

No es recomendable porque puede afectar a cualquier otro uso de git (sin relación) en su equipo Windows.

#### Bit de permisos de ejecución Unix

En Unix hay una bandera en los permisos de cada archivo que le dice al SO cuándo está permitida o no la ejecución del archivo. *git* a través de Cygwin soporta y está preparado para ese bit (incluso pensando que el sistema de archivos Windows NTFS no lo usa). Esto resulta a veces en que *git* encuentre diferencias "falso-positivo" en los permisos. El diff resultante sería algo como esto:

    diff --git ...
    old mode 100644
    new mode 100755
    

Recomendamos deshabilitar globalmente la comprobación de permisos en Windows para evitar el problema:

    git config --global core.fileMode false # deshabilita el bit de comprobación de ejecución globalmente para la máquina
    

Y para repositorios existentes que tienen este problema causado por una configuración local, agregar también:

    git config --unset core.filemode # elimina la opción local para este repositorio de aplicarlo globalmente
    git submodule foreach --recursive git config --unset core.filemode # elimina la opción local para todos los submódulos
    

## Información adicional

### Features / Issues {#features}

Se sabe que funcionan las siguientes funcionalidades (versión 2.0):

* Compilar y ejecutar SITL con jMAVSim con un rendimiento significativamente mejor que una MV (genera un binario nativo de windows **px4.exe**).
* Compilar y cargar compilaciones NuttX (por ejemplo: px4_fmu-v2 and px4_fmu-v4)
* Comprobar estilo con *astyle* (soporta el comando: `make format`)
* Autocompletado de linea de comandos
* ¡Instalador no-invasivo! El programa de instalación NO afecta a tu sistema ni a la ruta global (sólo modifica el directorio de instalación seleccionado, por ejemplo, **C:\PX4** y utiliza una ruta de acceso local temporal).
* El instalador puede actualizar a una nueva versión manteniendo los cambios personales dentro de la carpeta de la toolchain

Omisiones:

* Simulación: Gazebo y ROS no son compatibles
* Sólo compilaciones NuttX y JMAVSim/SITL son compatibles.
* [Problemas conocidos / informe de su problema](https://github.com/orgs/PX4/projects/6)

### Instalación Shell Script {#script_setup}

También puede instalar el entorno usando scripts de shell en el proyecto de Github.

1. Asegúrese de que tener [Git para Windows](https://git-scm.com/download/win) instalado.
2. Clone el repositorio https://github.com/PX4/windows-toolchain en la ubicación que desea instalar el toolchain. La nomenclatura y ubicación predeterminada se logra abriendo `Git Bash` y ejecutando:

    cd /c/
    git clone https://github.com/PX4/windows-toolchain PX4
    

1. Si desea instalar todos los componentes navegue a la carpeta recién clonada y haga doble clic en el script `install-all-components.bat` ubicado en la carpeta `toolchain`. If you only need certain components and want to safe Internet traffic and or disk space you can navigate to the different component folders like e.g. `toolchain\cygwin64` and click on the **install-XXX.bat** scripts to only fetch something specific.
2. Continúe con [Introducción](#getting_started) (o [Instrucciones de Uso](#usage_instructions)) 

### Manual de instalación (para desarrolladores de Toolchain) {#manual_setup}

This section describes how to setup the Cygwin toolchain manually yourself while pointing to the corresponding scripts from the script based installation repo. The result should be the same as using the scripts or MSI installer.

> **Nota** La toolchain va mejorando y por lo tanto, estas instrucciones no podrían cubrir cada detalle de todos los cambios futuros.

1. Cree las *carpetas*: **C:\PX4**, **C:\PX4\toolchain** y **C:\PX4\home**
2. Descargar el archivo *instalador Cygwin* [setup-x86_64.exe](https://cygwin.com/setup-x86_64.exe) desde la [Web oficial de Cygwin](https://cygwin.com/install.html)
3. Ejecute el archivo de instalación descargado
4. En el asistente elija instalar en la carpeta: **C:\PX4\toolchain\cygwin64**
5. Seleccione instalar el Cygwin base por defecto y la última versión disponible de los siguientes paquetes adicionales:

* **Categoría: Packagename**
* Devel:cmake (3.3.2 gives no deprecated warnings, 3.6.2 works but has the warnings)
* Devel:gcc-g++
* Devel:git
* Devel:make
* Devel:ninja
* Devel:patch
* Editors:xxd
* Editors:nano (a no ser que seas un pro en vim)
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
    
    > **Nota** No seleccione tantos paquetes como sea posible que no estén en esta lista, hay algunos que entran en conflicto e interrumpe la compilación.
    
    <span></span>
    
    > **Note** Eso es lo que hace [cygwin64/install-cygwin-px4.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/cygwin64/install-cygwin-px4.bat).

1. Escribir o copiar los **scripts de bash** [`run-console.bat`](https://github.com/MaEtUgR/PX4Toolchain/blob/master/run-console.bat) y [`setup-environment-variables.bat`](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/setup-environment-variables.bat).
    
    La razón para iniciar todas las herramientas de desarrollo a través de los script batch preparados es que preconfiguran el programa al inicio para usar el entorno Cygwin portable dentro de la carpeta de la toolchain. Esto se hace llamando siempre llamando primero a los scripts [**setup-environment-variables.bat**](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/setup-environment-variables.bat) y la aplicación deseada como la consola después de eso.
    
    El script de [`setup-environment-variables.bat`](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/setup-environment-variables.bat) localmente establece variables de entorno para el directorio de raíz del espacio de trabajo `PX4_DIR`, todas las ubicaciones de binarios, `PATH` y el directorio home del entorno unix `HOME`.

2. Añade los **paquetes de python** necesarios para la configuración abriendo la consola de la toolchain de Cygwin (doble clic en **run-console.bat**) y ejecutandolo
    
        pip2 install toml
        pip2 install pyserial
        pip2 install pyulog
        
    
    > **Nota** Es lo que hace [cygwin64/install-cygwin-python-packages.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/cygwin64/install-cygwin-python-packages.bat).

3. Descargar el [**compilador ARM GCC**](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads) como archivo zip de los binarios para Windows y descomprime el contenido a la carpeta `C:\PX4\toolchain\gcc-arm`.
    
    > **Nota** Esto es lo que hace la toolchain hace en: [gcc-arm/install-gcc-arm.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/gcc-arm/install-gcc-arm.bat).

4. Instala el JDK:
    
    * Descargar el [**Instalador de Kit de desarrollo Java**](http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html).
    * Because sadly there is no portable archive containing the binaries directly you have to install it.
    * Find the binaries and move/copy them to **C:\PX4\toolchain\jdk**.
    * You can uninstall the Kit from your Windows system again, we only needed the binaries for the toolchain.
    
    > **Note** This is what the toolchain does in: [jdk/install-jdk.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/jdk/install-jdk.bat).

5. Download [**Apache Ant**](https://ant.apache.org/bindownload.cgi) as zip archive of the binaries for Windows and unpack the content to the folder `C:\PX4\toolchain\apache-ant`.
    
    > **Tip** Make sure you don't have an additional folder layer from the folder which is inside the downloaded archive.
    
    <span></span>
    
    > **Note** This is what the toolchain does in: [apache-ant/install-apache-ant.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/apache-ant/install-apache-ant.bat).

6. Descarga, compila y añade *genromfs* a la ruta:
    
    * Clone the source code to the folder **C:\PX4\toolchain\genromfs\genromfs-src** with 
            cd /c/toolchain/genromfs
            git clone https://github.com/chexum/genromfs.git genromfs-src

* Compílalo con: 
    
        cd genromfs-src
         make all
    
    * Copy the resulting binary **genromfs.exe** one folder level out to: **C:\PX4\toolchain\genromfs**
    
    > **Note** This is what the toolchain does in: [genromfs/install-genromfs.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/genromfs/install-genromfs.bat).

1. Make sure all the binary folders of all the installed components are correctly listed in the `PATH` variable configured by [**setup-environment-variables.bat**](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/setup-environment-variables.bat).