# Windows Cygwin 工具链

该工具链非常轻便，而且容易安装和使用。 它是目前Windows环境下用于PX4开发的最新和最好的工具。

> **提示** 这是官方唯一支持的在Windows环境下开发PX4的工具链（它已经在集成测试系统中经过测试）

该工具链支持：

* 编译/上传 PX4到Nuttx目标(Pixhawk系列飞控)
* JMAVSim/SITL 仿真会获得比其他Windows工具链更好的性能
* 类型校验，轻便安装，完整的命令行支持和许多[其他特性](#features)

这篇文章将解释怎样下载和使用该环境，并且在需要的时候怎样扩展和更新(比如，使用其他的编译器)。

## 安装说明 {#installation}

1. Download the latest version of the ready-to-use MSI installer from: [Github releases](https://github.com/PX4/windows-toolchain/releases) or [Amazon S3](https://s3-us-west-2.amazonaws.com/px4-tools/PX4+Windows+Cygwin+Toolchain/PX4+Windows+Cygwin+Toolchain+0.9.msi) (fast download).
2. Run it, choose your desired installation location, let it install: ![jMAVSimOnWindows](../../assets/toolchain/cygwin_toolchain_installer.PNG)
3. 在安装结束后勾选*clone the PX4 repository, build and run simulation with jMAVSim*(这简化了你的开始准备工作)
    
    > **Note** If you missed this step you will need to [clone the PX4 Firmware repository manually](#getting_started).

## 入门指南 {#getting_started}

工具链使用专门配置的控制台(通过运行**run-console.bat**脚本)从而可以使用PX4编译命令

1. 进入到工具链的安装目录(默认**C:\PX4**)
2. 运行**run-console.bat**(双击)启动Cygwin bash控制台
3. 在控制台中运行克隆PX4 Firmware仓库命令
    
    > **注意**只需要克隆一次 如果你在安装程序最后选择了*clone the PX4 repository, build and run simulation with jMAVSim*，则可以跳过这一步。
    
    ```bash
    # 克隆 PX4 Firmware 仓库到 home 目录 & 同时并行加载子模块
    git clone --recursive -j8 https://github.com/PX4/Firmware.git
    ```
    
    你现在可以使用控制台中的Firmware仓库代码来编译PX4

4. 举例，要运行JMAVSim:
    
    ```bash
    # 进入Firmware仓库目录
    cd Firmware 
    # 使用JMAVSim编译并运行SITL模拟器来验证 
    make px4_sitl jmavsim
    ```
    
    控制台将会显示：
    
    ![jMAVSimOnWindows](../../assets/simulation/jmavsim_windows_cygwin.PNG)

下面[ 有关如何生成 PX4 的详细说明 ](../setup/building_px4.md) (或参阅下面的部分以了解更多常规用法说明)。

## 使用说明 {#usage_instructions}

安装目录 （默认位置： **C:\PX4**） 用于开启PX4 SITL（类Linux）命令行窗口的脚本文件： **run-console.bat**

> **Tip** [Manual Setup](#manual_setup) 部分解释了为什么需要使用该脚本以及它的工作原理。

普遍的工作流程都通过双击 **run-console. bat** 脚本来手动运行终端命令来启动控制台窗口。

### File Monitoring Tools vs Toolchain Speed

Antivirus and other background file monitoring tools can significantly slow down both installation of the toolchain and PX4 build times.

You may wish to halt them temporarily during builds (at your own risk).

### Windows & Git Special Cases

#### Windows CR+LF 对比 Unix LF 行结尾

We recommend that you force Unix style LF endings for every repository you're working with using this toolchain (and use an editor which preserves them when saving your changes - e.g. Eclipse or VS Code). Compilation of source files also works with CR+LF endings checked out locally, but there are cases in Cygwin (e.g. execution of shell scripts) that require Unix line endings (otherwise you get errors like `$'\r': Command not found.`). Luckily git can do this for you when you execute the two commands in the root directory of your repo:

    git config core.autocrlf false
    git config core.eol lf
    

If you work with this toolchain on multiple repositories you can also set these two configurations globally for your machine:

    git config --global ...
    

This is not recommended because it may affect any other (unrelated) git use on your Windows machine.

#### Unix 执行权限

Under Unix there's a flag in the permissions of each file that tells the OS whether or not the file is allowed to be executed. *git* under Cygwin supports and cares about that bit (even though the Windows NTFS file system does not use it). This often results in *git* finding "false-positive" differences in permissions. The resulting diff might look like this:

    diff --git ...
    old mode 100644
    new mode 100755
    

We recommend globally disabling the permission check on Windows to avoid the problem:

    git config --global core.fileMode false # disable execution bit check globally for the machine
    

For existing repositories that have this problem caused by a local configuration, additionally:

    git config --unset core.filemode # 移除当前存储库的局部配置，改用全局配置
    git submodule foreach --recursive git config --unset core.filemode # 移除所有子模块的局部配置
    

## 附加信息

### Features / Issues {#features}

The following features are known to work (version 2.0):

* 使用 jMAVSim 编译和运行 SITL, 其性能明显优于虚拟机 (Cygwin会生成一个本机 windows 二进制文件 ** px4.exe **)。
* 编译和上传 NuttX 二进制文件（例如：px4_fmu-v2 和 px4_fmu-v4）。
* 使用 * astyle * 进行格式检查 (支持命令: ` make format `)。
* 命令行自动补全。
* 绿色安装！ 安装程序不会影响您的系统和全局路径设置 (它只修改选定的安装目录, 例如 ** C:\PX4 \ ** 并使用临时本地路径变量)。
* 安装程序支持更新到最新版本, 同时保持您的个人更改在工具链文件夹中。

Omissions:

* Simulation: Gazebo and ROS are not supported.
* 仅支持 NuttX 和 JMAVSim/SITL 编译。
* [Known problems](https://github.com/orgs/PX4/projects/6) (Also use to report issues).

### Shell Script Installation {#script_setup}

You can also install the environment using shell scripts in the Github project.

1. 请确保安装了 [ Windows Git ](https://git-scm.com/download/win)。
2. 将代码仓库 https://github.com/PX4/windows-toolchain 克隆到要安装工具链的位置。 打开 ` Git Bash ` 并执行以下操作，打开后会自动进入默认的安装目录:

    cd /c/
    git clone https://github.com/PX4/windows-toolchain PX4
    

1. 如果要安装所有组件, 请进入到新克隆的代码仓库文件夹, 然后双击位于文件夹 `toolchain`目录中的脚本 ` install-all-components.bat`。 如果您只需要某些组件并希望占用有限的Internet 数据和磁盘空间, 则可以进入到不同的组件文件夹, 如 ` toolchain\cygwin64 `, 然后单击 ** install-XXX.bat ** 脚本以获取特定的内容。
2. 继续 [ 入门指南 ](#getting_started) (或 [ 使用说明 ](#usage_instructions)) 

### Manual Installation (for Toolchain Developers) {#manual_setup}

This section describes how to setup the Cygwin toolchain manually yourself while pointing to the corresponding scripts from the script based installation repo. The result should be the same as using the scripts or MSI installer.

> **注意：** 因为工具链的更新，下述指令可能无法涵盖未来所有更改的每个细节。

1. 创建 * 文件夹 *: ** C:\PX4 \ **、** C:\PX4\toolchain \ ** 和 ** C:\PX4\home \ **
2. 从 [ Cygwin 官方网站 ](https://cygwin.com/install.html) 下载 * Cygwin 安装程序 * 文件 [ official Cygwin website ](https://cygwin.com/setup-x86_64.exe)
3. 运行下载的安装程序文件
4. 在安装向导中选择安装到文件夹中: ** C:\PX4\toolchain\cygwin64 \ **
5. 选择安装默认的 Cygwin 基础包和以下附加包的最新可用版本:

* **目录:安装包名**
* Devel:cmake (3.3.2 正常工作无告警, 3.6.2有告警但能够正常工作)
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
    
    使用预先准备好的批处理脚本启动开发环境的原因是，这些脚本预配置了程序使用工具链所在目录下的绿色版 Cygwin 环境变量。 This is done by always first calling the script [**setup-environment.bat**](https://github.com/PX4/windows-toolchain/blob/master/toolchain/scripts/setup-environment.bat) and the desired application like the console after that.
    
    The script [setup-environment.bat](https://github.com/PX4/windows-toolchain/blob/master/toolchain/scripts/setup-environment.bat) locally sets environmental variables for the workspace root directory `PX4_DIR`, all binary locations `PATH`, and the home directory of the unix environment `HOME`.

2. 通过执行 Cygwin 工具链控制台 (双击 ** run-console.bat **) 脚本, 向安装程序添加必要的 ** python 包 **
    
        pip2 install toml 
        pip2 install pyserial 
        pip2 install pyulog
        
    
    > ** 注意 **这就是 [ cygwin64/install-cygwin-pxbat ](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/cygwin64/install-cygwin-python-packages.bat) 所做的工作。

3. 下载 [**ARM GCC 编译器**](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads) zip 存档，并将内容解压缩到文件夹 `C:\PX4\toolchain\gcc-arm`。
    
    > ** 注意 **这就是工具链在 [ gcc-arm/install-gcc-arm.bat ](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/gcc-arm/install-gcc-arm.bat) 脚本中所做的工作。

4. 安装 JDK
    
    * 下载 [** Java Development Kit Installer **](http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html)。
    * 因为不幸的是, 没有绿色的归档文件直接包含二进制文件, 所以您必须安装它。
    * 查找二进制文件并将其移动/复制到 ** C:\PX4\toolchain\jdk **。
    * 您可以再次从 Windows 系统中卸载该JDK工具包, 我们只需要工具链的二进制文件。
    
    > ** 注意 **这就是工具链在 [ gcc-arm/install-jdk.bat ](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/jdk/install-jdk.bat) 脚本中所做的工作。

5. 下载 [**Apache Ant**](https://ant.apache.org/bindownload.cgi) zip 存档, 并将内容解压缩到文件夹 `C:\PX4\toolchain\apache-ant`。
    
    > ** 提示 **请确保您没有从下载的归档文件内的文件夹中添加其他文件夹层。
    
    <span></span>
    
    > ** 注意 **这就是工具链在 [ gcc-arm/install-apache-ant.bat ](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/apache-ant/install-apache-ant.bat) 脚本中所做的工作。

6. 下载、编译并将 * genromfs *添加到环境变量:
    
    * 将源代码克隆到文件夹 ** C:\PX4\toolchain\genromfs\genromfs-src ** 中, 
            cd /c/toolchain/genromfs
            git clone https://github.com/chexum/genromfs.git genromfs-src

* 使用如下命令进行编译： 
    
        cd genromfs-src
         make all
    
    * 将包含有生成的二进制 ** genromfs.exe ** 的文件夹复制到: ** C:\PX4\toolchain\genromfs **
    
    > ** 注意： **这就是工具链在 [ gcc-arm/install-genromfs.bat ](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/genromfs/install-genromfs.bat) 脚本中所做的工作。

1. Make sure all the binary folders of all the installed components are correctly listed in the `PATH` variable configured by [**setup-environment.bat**](https://github.com/PX4/windows-toolchain/blob/master/toolchain/scripts/setup-environment.bat).