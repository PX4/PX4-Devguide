# Windows Cygwin Toolchain

## Ready to use MSI Installer Download

Latest Download:

* [PX4 Windows Cygwin Toolchain 0.2 Download](https://s3-us-west-2.amazonaws.com/px4-tools/PX4+Windows+Cygwin+Toolchain/PX4+Windows+Cygwin+Toolchain+0.2.msi) (09.05.2018)

Legacy Versions (**deprecated**):

* [PX4 Windows Cygwin Toolchain 0.1 Download](https://s3-us-west-2.amazonaws.com/px4-tools/PX4+Windows+Cygwin+Toolchain/PX4+Windows+Cygwin+Toolchain+0.1.msi) (23.02.2018)

## Support / Known Problems

Tested and working:

* Building and running SITL with jMAVsim, generates a native windows binary px4.exe and has a lot better prformance than a VM
* Building and uploading NuttX builds like px4fmu-v2 and px4fmu-v4
* Style check with astyle with the command make format
* Command line auto completion
* The installer does **NOT** screw up your system and global path, it only modifies the selected installation directory e.g. `C:\PX4\` and uses a temporary local path
* The installer supports updating to a new version keeping your personal changes inside the toolchain folder

[Known problems / Report your issue](https://github.com/orgs/PX4/projects/6)

## Installation Instructions

1. Download the latest version of the ready to use MSI installer above
1. Run it, choose your desired installation location, let it install
    ![jMAVSimOnWindows](../../assets/toolchain/cygwin_toolchain_installer.PNG)
1. Make sure to tick the box in the end of the installation to clone the PX4 repository, build and run simulation with jMAVsim this simplifies the process to get you started. 

> **Note** Don't worry if you missed the opportunity or the script failed (e.g. because you didn't have a working internet connection). You can also [do that step manually on the console afterwards](#getting_started).

## Usage Instructions {#usage_instructions}

After the installation when you browse to the directory you installed the toolchain to (default `C:\PX4\`) you'll find the following batch scripts which start up a console/terminal or different IDEs inside the Cygwin toolchain environment:

* **`run-console.bat`**

    To start the POSIX (linux like) bash console.

* **`run-eclipse.bat`**

    To start the as of Toolchain version 0.2 built in portable [eclipse for C++ IDE](http://www.eclipse.org/downloads/eclipse-packages/).

* **`run-vscode.bat`**

    To start the not included [Visual Studio Code IDE](https://code.visualstudio.com/) from its default install directory `C:\Program Files\Microsoft VS Code`

> **Tip** Further details on why you need to use the scripts and how it all works [explained below](#manual_setup).

<span></span>
> **Tip** You can create desktop shortcuts to the batch scripts to have easier access, the installer as of toolchain version 0.2 does not yet create them for you.

The ordinary workflow consists of starting a console window by douple clicking on the `run-console.bat` script to manually run terminal commands and starting your favorite IDE with its corresponding `run-XXX.bat` script to edit code and possibly also run builds. 

### Getting Started {#getting_started}

Ticking the box to clone and run simulation at the end of the installer opens the Cygwin console and runs the following commands:
```
# clones PX4 repository into the home folder & loads submodules in parallel
git clone --recursive -j8 https://github.com/PX4/Firmware.git
# switches into the Firmware repository folder
cd Firmware
# builds and runs SITL simulation with jMAVSim to test the setup
make posix jmavsim
```

> **Note** Cloning only needs to be done once, if you ticked the box to clone at the installer or just updated your toolchain it's already done and you can skip the fist command.

![jMAVSimOnWindows](../../assets/simulation/jmavsim_windows_cygwin.PNG)

Continue at this stage with [the detailed instructions on how to build PX4](../setup/building_px4.md).

### Windows & Git Special Cases

#### Windows CR+LF vs Unix LF Line Endings
It is recommended that you force unix style LF endings for every repository you're working with using this Toolcahin and use an editor which preserves them when saving your changes like eclipse or VS Code.
Compilation of source files also works with CR+LF endings checked out locally but there are cases in Cygwin like execution of shell scripts that require unix line endings otherwise you get errors like `$'\r': Command not found.`.
Luckily git can do this for you when you execute the two commands in the root directory of your repo:
```
git config core.autocrlf false
git config core.eol lf
```
If you work with this Toolchain on multiple repositories you can also set these two configurations globally for your machine by `git config --global ...` but this will apply system wide and hence also in other unrelated git usages on your Windows and is not recommended.

#### Unix permissions execution bit
Under unix there's a flag in the permissions of each file which tells the OS if the file is allowed to be executed or not. Windows has a different permission system and no such bit but git especially under cygwin supports and cares about that bit.
This often results in git finding difffereces in permissions even if there is no real diff which looks like this:
```
diff --git ...
old mode 100644
new mode 100755
```

It's recommended to disable this functionality by executing `git config core.fileMode false` in every repo you use with this toolchain.

## Shell Script installation {#script_setup}
1. Make sure you have Git for Windows installed. You can get it from https://git-scm.com/download/win.
1. Clone the repository https://github.com/MaEtUgR/PX4Toolchain to the location you want to install the toolchain. Default location and naming is achieved by opening the `Git Bash` and executing:
```
cd /c/
git clone https://github.com/MaEtUgR/PX4Toolchain PX4
```
1. If you want to install all components navigate to the freshly cloned folder and double click on the script `install-all-components.bat` located in the folder `toolchain`. If you only need certain components and want to safe internet traffic and or disk space you can navigate to the different component folders like e.g. `toolchain\cygwin64` and click on the `install-XXX.bat` scripts to only fetch something specific.
1. Continue with [Usage Instructions](#usage_instructions) and [Getting Started](#getting_started)

## Manual Installation (for Toolchain Developers) {#manual_setup}

This section describes how to setup the Cygwin toolchain manually yourself while pointing to the corresponding scripts from the script based installation repo. The result should be the same as using the scripts or MSI installer.

> **Note** The Toolchain gets maintained and hence these instructions might not cover every detail of all the future changes.

1. Create the **folders** `C:\PX4\`, `C:\PX4\toolchain\` and `C:\PX4\home\`
1. Download the **Cygwin installer** file [setup-x86_64.exe](https://cygwin.com/setup-x86_64.exe) from the [official Cygwin website](https://cygwin.com/install.html)
1. Run the downloaded setup file
1. In the wizard choose to install into the folder `C:\PX4\toolchain\cygwin64\`
1. Select to install the default Cygwin base and the newest available version of the following additional packages:

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
    * Archive:unzip
    * Utils:astyle
    * Shells:bash-completion
    * Web:wget

    > **Note** Do not select as many packages as possible which are not on this list, there are some which conflict and break the builds.

    <span></span>
    > **Note** That's what [cygwin64/install-cygwin-px4.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/cygwin64/install-cygwin-px4.bat) does.

1. 
    Write up or copy the **batch scripts** [`run-console.bat`](https://github.com/MaEtUgR/PX4Toolchain/blob/master/run-console.bat) and [`setup-environment-variables.bat`](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/setup-environment-variables.bat). 
    
    The reason to start all the development tools through the prepared batch scripts is they preconfigure the starting program to use the local, portable Cygwin environment inside the toolchain's folder. This is done by always first calling the script [`setup-environment-variables.bat`](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/setup-environment-variables.bat) and the desired application like the console after that.

    The script [`setup-environment-variables.bat`](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/setup-environment-variables.bat)
    locally sets environmental variables for the workspace root directory `PX4_DIR`, all binary locations `PATH`, and the home directory of the unix environment `HOME`.

1. 
    Add necessary **python packages** to your setup by opening the Cygwin toolchain console (double clicking `run-console.bat`) and executing
    ```
    pip2 install toml
    pip2 install pyserial
    pip2 install pyulog
    ```
    
    > **Note** That's what [cygwin64/install-cygwin-python-packages.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/cygwin64/install-cygwin-python-packages.bat) does.

1. 
    Download the [**ARM GCC compiler**](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads) as zip archive of the binaries for Windows and unpack the content to the folder `C:\PX4\toolchain\gcc-arm`.

    > **Note** That's what [gcc-arm/install-gcc-arm.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/gcc-arm/install-gcc-arm.bat) does.

1. 
    * Download the [**Java Development Kit Installer**](http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html).
    * Because sadly there is no portable archive containing the binaries directly you have to install it.
    *  Find the binaries and move/copy them to `C:\PX4\toolchain\jdk`.
    * You can uninstall the Kit from your Windows system again, we only needed the binaried for the toolchain.

    > **Note** That's what [jdk/install-jdk.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/jdk/install-jdk.bat) does.

1. 
    Download [**Apache Ant**](https://ant.apache.org/bindownload.cgi) as zip archive of the binaries for Windows and unpack the content to the folder `C:\PX4\toolchain\apache-ant`.

    > **Tip** Make sure you don't have an additional folder layer from the folder which is inside the downloaded archive.

    <span></span>
    > **Note** That's what [apache-ant/install-apache-ant.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/apache-ant/install-apache-ant.bat) does.

1. 
    Download, build and add **genromfs** to the path:
    * Clone the source code to the folder `C:\PX4\toolchain\genromfs\genromfs-src` with
        ```
        cd /c/toolchain/genromfs
        git clone https://github.com/chexum/genromfs.git genromfs-src
        ```

    * compile it with
        ```
        cd genromfs-src
        make all
        ```

    * Copy the resulting binary `genromfs.exe` one folder level out to `C:\PX4\toolchain\genromfs`

    > **Note** That's what [genromfs/install-genromfs.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/genromfs/install-genromfs.bat) does.

1. 
    Make sure all the binary folders of all the isntalled components are correctly listed in the `PATH` variable configured by [`setup-environment-variables.bat`](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/setup-environment-variables.bat).
