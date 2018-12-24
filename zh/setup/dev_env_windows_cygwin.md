# Windows Cygwin 工具链

该工具链非常轻便，而且容易安装和使用。 它是目前Windows环境下用于PX4开发的最新和最好的工具。

> **提示** 这是官方唯一支持的在Windows环境下开发PX4的工具链（它已经在集成测试系统中经过测试）

该工具链支持：

* 编译/上传 PX4到Nuttx目标(Pixhawk系列飞控)
* JMAVSim/SITL 仿真会获得比其他Windows工具链更好的性能
* 类型校验，轻便安装，完整的命令行支持和许多[其他特性](#features)

这篇文章将解释怎样下载和使用该环境，并且在需要的时候怎样扩展和更新(比如，使用其他的编译器)。

<!-- Legacy Versions (**deprecated**):

* [PX4 Windows Cygwin Toolchain 0.4 Download](https://s3-us-west-2.amazonaws.com/px4-tools/PX4+Windows+Cygwin+Toolchain/PX4+Windows+Cygwin+Toolchain+0.4.msi) (18.09.2018)
* [PX4 Windows Cygwin Toolchain 0.3 Download](https://s3-us-west-2.amazonaws.com/px4-tools/PX4+Windows+Cygwin+Toolchain/PX4+Windows+Cygwin+Toolchain+0.3.msi) (25.07.2018)
* [PX4 Windows Cygwin Toolchain 0.2 Download](https://s3-us-west-2.amazonaws.com/px4-tools/PX4+Windows+Cygwin+Toolchain/PX4+Windows+Cygwin+Toolchain+0.2.msi) (09.05.2018)
* [PX4 Windows Cygwin Toolchain 0.1 Download](https://s3-us-west-2.amazonaws.com/px4-tools/PX4+Windows+Cygwin+Toolchain/PX4+Windows+Cygwin+Toolchain+0.1.msi) (23.02.2018)
-->

## 安装说明 {#installation}

1. 从 [Github](https://github.com/PX4/windows-toolchain/releases) 或者 [S3](https://s3-us-west-2.amazonaws.com/px4-tools/PX4+Windows+Cygwin+Toolchain/PX4+Windows+Cygwin+Toolchain+0.5.msi) 下载最新的MSI安装文件。
2. 运行它，选择你需要的安装路径，执行安装 ![jMAVSimOnWindows](../../assets/toolchain/cygwin_toolchain_installer.PNG)
3. 在安装结束后勾选*clone the PX4 repository, build and run simulation with jMAVSim*(这简化了你的开始准备工作)
    
    > **注意**如果你错过了这一步，你需要[手动克隆PX4 Firmware代码仓库](#getting_started)

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

安装目录 （默认位置： **C:\PX4**） 用于开启PX4 SITL（类Linux）命令行窗口的脚本文件： **run-console.bat**

> ** 提示 **[ Manual Setup ](#manual_setup) 部分解释了为什么需要使用该脚本以及它的工作原理。

普遍的工作流程都通过双击 ** run-console. bat ** 脚本来手动运行终端命令来启动控制台窗口。

### Windows & Git 特殊情况

#### Windows CR + LF 对比 Unix LF 行结尾

我们建议您所有的代码仓库都强制使用Unix的LF行结尾，并以此运行工具链（并且使用编辑器可以按照此格式保存您所做的修改 - 譬如 Eclipse 或者 VS Code）。 虽然编译以 CR+LF 行为结尾的本地源代码也是可行的， 但 Cygwin在某些情况下（如执行 shell 脚本）仍要求文件以 Unix 行结尾 (否则你会收到类似 `$'\r': Command not found.` 的错误信息）。 幸运的是, 只需要在代码仓库的根目录执行以下两条命令就可以让 git 自动为你完成此操作：

    git config core.autocrlf false
    git config core.eol lf
    

如果需要在多个代码仓库中使用此工具链,，你可以为你的计算机在全局范围内设置这两种配置：

    git config --global ...
    

但我们并不建议这样做, 因为它可能会影响 Windows 计算机上的任何其他 (无关) git 使用。

#### Unix 执行权限

在 Unix 下, 每个文件的权限中都有一个标志位, 它会告诉操作系统是否允许执行该文件。 Cygwin 下的 * git * 支持并遵守该标识位 (尽管 Windows 平台的NTFS文件系统并不使用该标志位)。 这一差异通常会导致 *git* 发现权限中的 "假阳性（false-positive）" 差异。 生成的差异可能如下所示:

    diff --git ...
    old mode 100644
    new mode 100755
    

我们建议在 windows 平台上全局禁用权该限检查以避免这个问题：

    git config --global core.fileMode false # disable execution bit check globally for the machine
    

对于由局部配置引起此问题的现有存储库，你可以使用如下命令：

    git config --unset core.filemode # 移除当前存储库的局部配置，改用全局配置
    git submodule foreach --recursive git config --unset core.filemode # 移除所有子模块的局部配置
    

## 附加信息

### 特性/问题 {#features}

以下已知正常功能 (版本 2.0):

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

### Shell 脚本安装 {#script_setup}

你还可以使用 Github 项目中的 shell 脚本进行开发环境的安装。

1. 请确保安装了 [ Windows Git ](https://git-scm.com/download/win)。
2. 将代码仓库 https://github.com/PX4/windows-toolchain 克隆到要安装工具链的位置。 打开 ` Git Bash ` 并执行以下操作，打开后会自动进入默认的安装目录:

    cd /c/
    git clone https://github.com/PX4/windows-toolchain PX4
    

1. 如果要安装所有组件, 请进入到新克隆的代码仓库文件夹, 然后双击位于文件夹 `toolchain`目录中的脚本 ` install-all-components.bat`。 如果您只需要某些组件并希望占用有限的Internet 数据和磁盘空间, 则可以进入到不同的组件文件夹, 如 ` toolchain\cygwin64 `, 然后单击 ** install-XXX.bat ** 脚本以获取特定的内容。
2. 继续 [ 入门指南 ](#getting_started) (或 [ 使用说明 ](#usage_instructions)) 

### 手动安装 (对于开发人员) {#manual_setup}

This section describes how to setup the Cygwin toolchain manually yourself while pointing to the corresponding scripts from the script based installation repo. The result should be the same as using the scripts or MSI installer.

> **Note** The toolchain gets maintained and hence these instructions might not cover every detail of all the future changes.

1. 创建 * 文件夹 *: ** C:\PX4 \ **、** C:\PX4\toolchain \ ** 和 ** C:\PX4\home \ **
2. 从 [ Cygwin 官方网站 ](https://cygwin.com/install.html) 下载 * Cygwin 安装程序 * 文件 [ official Cygwin website ](https://cygwin.com/setup-x86_64.exe)
3. 运行下载的安装程序文件
4. 在安装向导中选择安装到文件夹中: ** C:\PX4\toolchain\cygwin64 \ **
5. 选择安装默认的 Cygwin 基础包和以下附加包的最新可用版本:

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

1. 编写或复制 ** 批处理脚本 ** [` run-console.bat `](https://github.com/MaEtUgR/PX4Toolchain/blob/master/run-console.bat) 和 [` setup-environment-variables.bat `](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/setup-environment-variables.bat)。
    
    The reason to start all the development tools through the prepared batch script is they preconfigure the starting program to use the local, portable Cygwin environment inside the toolchain's folder. 这是通过始终首先调用脚本 [** setup-environment-variables.bat **](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/setup-environment-variables.bat) 和所需的应用程序 (如控制台) 来完成的。
    
    The script [`setup-environment-variables.bat`](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/setup-environment-variables.bat) locally sets environmental variables for the workspace root directory `PX4_DIR`, all binary locations `PATH`, and the home directory of the unix environment `HOME`.

2. 通过执行 Cygwin 工具链控制台 (双击 ** run-console.bat **) 脚本, 向安装程序添加必要的 ** python 包 **
    
        pip2 install toml 
        pip2 install pyserial 
        pip2 install pyulog
        
    
    > ** 注意 **这就是 [ cygwin64/install-cygwin-pxbat ](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/cygwin64/install-cygwin-python-packages.bat) 所做的工作。

3. 下载 [** ARM GCC 编译器 **](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads) zip 存档, 并将内容解压缩到文件夹 ` C:\PX4\toolchain\gcc-arm `。
    
    > ** 注意 **这就是工具链在 [ gcc-arm/install-gcc-arm.bat ](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/gcc-arm/install-gcc-arm.bat) 脚本中所做的工作。

4. 安装 JDK
    
    * 下载 [** Java Development Kit Installer **](http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html)。
    * 因为不幸的是, 没有绿色的归档文件直接包含二进制文件, 所以您必须安装它。
    * 查找二进制文件并将其移动/复制到 ** C:\PX4\toolchain\jdk **。
    * 您可以再次从 Windows 系统中卸载该JDK工具包, 我们只需要工具链的二进制文件。
    
    > ** 注意 **这就是工具链在 [ gcc-arm/install-jdk.bat ](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/jdk/install-jdk.bat) 脚本中所做的工作。

5. 下载 [** Apache Ant **](https://ant.apache.org/bindownload.cgi) zip 存档, 并将内容解压缩到文件夹 ` C:\PX4\toolchain\apache-ant `。
    
    > ** 提示 **请确保您没有从下载的归档文件内的文件夹中添加其他文件夹层。
    
    <span></span>
    
    > ** 注意 **这就是工具链在 [ gcc-arm/install-apache-ant.bat ](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/apache-ant/install-apache-ant.bat) 脚本中所做的工作。

6. 下载、编译并将 * genromfs *添加到环境变量:
    
    * 将源代码克隆到文件夹 ** C:\PX4\toolchain\genromfs\genromfs-src ** 中, 
            cd /c/toolchain/genromfs
            git clone https://github.com/chexum/genromfs.git genromfs-src

* Compile it with: 
    
        cd genromfs-src
         make all
    
    * Copy the resulting binary **genromfs.exe** one folder level out to: **C:\PX4\toolchain\genromfs**
    
    > **Note** This is what the toolchain does in: [genromfs/install-genromfs.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/genromfs/install-genromfs.bat).

1. 确保所有已安装组件的二进制文件夹都正确配置在 [** setup-environment-variables.bat **](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/setup-environment-variables.bat) 配置的 ` 环境变量 ` 中。