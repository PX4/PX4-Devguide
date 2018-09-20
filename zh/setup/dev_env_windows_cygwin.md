# Windows Cygwin 工具链

该工具链非常轻便，而且容易安装和使用。 它是目前Windows环境下用于PX4开发的最新和最好的工具。

> **提示** 这是官方唯一支持的在Windows环境下开发PX4的工具链（它已经在集成测试系统中经过测试）

该工具链支持：

* 编译/上传 PX4到Nuttx目标(Pixhawk系列飞控)
* JMAVSim/SITL 仿真会获得比其他Windows工具链更好的性能
* 类型校验，轻便安装，完整的命令行支持和许多[其他特性](#features)

这篇文章将解释怎样下载和使用该环境，并且在需要的时候怎样扩展和更新(比如，使用其他的编译器)。

<!--
## Ready to use MSI Installer Download {#installation}

Latest Download: [PX4 Windows Cygwin Toolchain 0.3 Download](https://s3-us-west-2.amazonaws.com/px4-tools/PX4+Windows+Cygwin+Toolchain/PX4+Windows+Cygwin+Toolchain+0.3.msi) (25.07.2018)


Legacy Versions (**deprecated**):

* [PX4 Windows Cygwin Toolchain 0.3 Download](https://s3-us-west-2.amazonaws.com/px4-tools/PX4+Windows+Cygwin+Toolchain/PX4+Windows+Cygwin+Toolchain+0.3.msi) (25.07.2018)
* [PX4 Windows Cygwin Toolchain 0.2 Download](https://s3-us-west-2.amazonaws.com/px4-tools/PX4+Windows+Cygwin+Toolchain/PX4+Windows+Cygwin+Toolchain+0.2.msi) (09.05.2018)
* [PX4 Windows Cygwin Toolchain 0.1 Download](https://s3-us-west-2.amazonaws.com/px4-tools/PX4+Windows+Cygwin+Toolchain/PX4+Windows+Cygwin+Toolchain+0.1.msi) (23.02.2018)
-->

## 安装说明

1. 下载最新的MSI安装文件：[PX4 Windows Cygwin Toolchain 0.3 Download](https://s3-us-west-2.amazonaws.com/px4-tools/PX4+Windows+Cygwin+Toolchain/PX4+Windows+Cygwin+Toolchain+0.3.msi)(25.07.2018)
2. 运行它，选择你需要的安装路径，执行安装 ![jMAVSimOnWindows](../../assets/toolchain/cygwin_toolchain_installer.PNG)
3. 在安装结束后勾选*clone the PX4 repository, build and run simulation with jMAVSim*(这简化了你的开始准备工作)
    
    > **注意**如果你错过了这一步，你需要[手动克隆PX4 Firmware库](#getting_started)

## 入门指南 {#getting_started}

工具链使用专门配置的控制台(通过运行**run-console.bat**脚本)从而可以使用PX4编译命令

1. 进入到工具链的安装目录(默认**C:\PX4**)
2. 运行**run-console.bat**(双击)启动Cygwin bash控制台
3. 在控制台中运行克隆PX4 Firmware仓库命令
    
    > **注意**只需要克隆一次 如果你在安装程序最后选择了*clone the PX4 repository, build and run simulation with jMAVSim*，则可以跳过这一步。
    
    ```bash
    克隆PX4 Firmware仓库到home目录& 同时并行加载子模块
    git clone --recursive -j8 https://github.com/PX4/Firmware.git
    ```
    
    你现在可以使用控制台中的Firmware仓库代码来编译PX4

4. 举例，要运行JMAVSim:
    
    ```bash
    # 进入Firmware仓库目录
    cd Firmware 
    # 使用JMAVSim编译并运行SITL模拟器来验证 
    make posix jmavsim
    ```
    
    控制台将会显示：
    
    ![jMAVSimOnWindows](../../assets/simulation/jmavsim_windows_cygwin.PNG)

下面[ 有关如何生成 PX4 的详细说明 ](../setup/building_px4.md) (或参阅下面的部分以了解更多常规用法说明)。

## 使用说明 {#usage_instructions}

安装目录(默认：**C:\PX4**)包含了用于启动控制台窗口和包含Cygwin工具链的其他开发工具的脚本。 下面是脚本列表。

* ** 运行控制台. bat **-启动 POSIX (类似 linux) bash 控制台。
* ** 运行 eclipse **-启动基于 [ eclipse 的 c++ IDE ](http://www.eclipse.org/downloads/eclipse-packages/)。
* ** 运行 vscode **-从默认安装目录: ` C:\Program File \ Microsoft VS Code `启动 [ Visual Studio Code IDE ](https://code.visualstudio.com/) (必须单独安装),

> ** 提示 **[ 手动设置 ](#manual_setup) 部分解释了为什么需要使用脚本以及它的工作原理。

<span></span>

> ** 提示 **您可以为批处理脚本创建桌面快捷方式, 以便更方便地访问, 因为安装程序可能未能为您创建它们 (如0.2的工具链版本)。

普通工作流程包括通过双击 ** run-console. bat ** 脚本来手动运行终端命令来启动控制台窗口。 更喜欢 IDE开发环境 的开发人员可以使用相应的 ** run- XXX. bat ** 脚本启动它, 以编辑代码/运行生成。

### Windows & Git 特殊情况

#### Windows CR + LF 对比 Unix LF 行结尾

我们建议您所有的代码仓库都强制使用Unix的LF行结尾，并以此运行工具链(并且使用编辑器可以按照此格式保存您所做的修改 - 譬如 Eclipse 或者 VS Code) 源文件也可以兼容 CR+LF 行结尾并保存在本地, 但在 Cygwin (如 shell 脚本的执行) 中需要 Unix 行结尾 (否则您会收到类似 ` $ ' \r ': 未找到命令的错误. `。 幸运的是, 当您在代码仓库的根目录中执行两个命令时, git 可以为您强制转换:

    git config core.autocrlf false
    git config core.eol lf
    

如果您在多个代码仓库中使用此工具链, 还可以为您的计算机在全局范围内设置这两种配置:

    git config --global ...
    

建议不要这样做, 因为它可能会影响 Windows 计算机上的任何其他 (无关) git 使用。

#### Unix 执行权限

Under Unix there's a flag in the permissions of each file which tells the OS whether or not the file is allowed to be executed. *git* under Cygwin supports and cares about that bit (even though Windows has a different permission system). This often results in *git* finding differences in permissions even if there is no real diff which looks like this:

    diff --git ...
    old mode 100644
    new mode 100755
    

We recommend disabling this functionality by executing `git config core.fileMode false` in every repo where you use with this toolchain.

## Additional Information

### Features / Issues {#features}

The following features are known to work (version 2.0):

* Building and running SITL with jMAVSim with significantly better performance than a VM (it generates a native windows binary **px4.exe**).
* Building and uploading NuttX builds (e.g.: px4fmu-v2 and px4fmu-v4)
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