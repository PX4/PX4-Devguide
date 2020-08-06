# 윈도우 Cygwin 툴체인

이 툴체인은 이식이 용이하고, 설치하기 쉬우며, 사용하기에도 간편합니다. 윈도우에서 PX4 개발에 활용하는 가장 최신의, 최상의 기능과 성능을 발휘하는 툴체인입니다.

> **Tip** 윈도우에서 PX4 빌드에 활용하는 툴체인 중, 유일하게 공식적으로 지원하는 툴체인입니다(예: 지속 통합 시스템에서 시험해보았습니다).

툴체인에서 지원하는 기능은 다음과 같습니다:

* PX4를 NuttX 대상(픽스호크 계열 조종기)에 빌드하고 업로드합니다
* jMAVSim/SITL 모의시험 환경의 성능이 다른 윈도우 툴체인을 사용할 때보다 비약적으로 개선됩니다.
* 코드 모양새 검사, 간편 설치 관리자, 명령행 자동 완성 등의 [기타 수많은 기능](#features)을 내포하고 있습니다.

이 주제에서는 환경을 다운로드하고 활용하는 방법, 필요한 경우 기능을 확장하고 업데이트하는 방법(예: 다른 컴파일러 활용)을 설명하도록 하겠습니다.

## 설치 방법 {#installation}

1. 바로 사용할 수 있는 MSI 설치 프로그램의 최신 버전을 [Github 릴리스](https://github.com/PX4/windows-toolchain/releases) 또는 [아마존 S3](https://s3-us-west-2.amazonaws.com/px4-tools/PX4+Windows+Cygwin+Toolchain/PX4+Windows+Cygwin+Toolchain+0.9.msi) (다운로드 속도 빠름) 서버에서 받으십시오.
2. 실행하고, 원하는 위치를 선택한 후, 진행하십시오:![jMAVSimOnWindows](../../assets/toolchain/cygwin_toolchain_installer.PNG)
3. 설치 과정 마지막에서 *clone the PX4 repository, build and run simulation with jMAVSim*의 확인 상자에 표시하십시오(이 동작은 시작할 때 과정을 단순화합니다).
    
    > **Note** 이 단계를 놓쳤다면 [PX4 펌웨어 저장소를 직접 가져와야합니다](#getting_started).

## 시작하기 {#getting_started}

툴체인은 별도로 설정한 콘솔 창(**run-console.bat** 스크립트 실행)을 사용하며, 이 콘솔창에서 PX4 빌드에 사용할 일반 명령을 호출할 수 있습니다:

1. 툴체인 설치 디렉터리를 탐색하십시오 (기본 위치 **C:\PX4**)
2. **run-console.bat** 을 실행(두번 누르기)하여 Cygwin 배시 콘솔을 실행하십시오
3. PX4 펌웨어 저장소를 콘솔에서 가져(clone)오십시오:
    
    > **Note** 가져오는 동작은 한번이면 됩니다! *clone the PX4 repository, build and run simulation with jMAVSim* 옵션을 설치 관리자에서 표시했다면 이 과정을 건너뛰십시오.
    
    ```bash
    # Clone PX4 Firmware repository into the home folder & loads submodules in parallel
    git clone --recursive -j8 https://github.com/PX4/Firmware.git
    ```
    
    이제 PX4를 빌드할 콘솔과 펌웨어 저장소를 활용할 수 있습니다.

4. 예를 들어 jMAVSim을 실행하려면:
    
    ```bash
    # Navigate to Firmware repo
    cd Firmware
    # Build and runs SITL simulation with jMAVSim to test the setup
    make px4_sitl jmavsim
    ```
    
    위 명령을 실행하면 화면이 나타납니다:
    
    ![jMAVSimOnWindows](../../assets/simulation/jmavsim_windows_cygwin.PNG)

[PX4를 빌드하는 자세한 방법](../setup/building_px4.md)으로 계속 진행하십시오(또는 바로 아래 절에서 좀 더 일반적인 사용 방법을 살펴보십시오).

## 사용 방법 {#usage_instructions}

설치 디렉터리(기본 위치:**C:\PX4**)에는 PX4 SITL(리눅스 유사) 배시 콘솔을 실행하는 배치 스크립트 **run-console.bat** 파일이 들어있습니다.

> **Tip** [수동 설치](#manual_setup) 절에서는 왜 스크립트를 사용해야 하는지 각각의 모든 과정이 어떤 동작을 하는지 설명합니다.

일반적인 과정은 **run-console.bat** 스크립트를 두번 눌러 터미널 명령을 직접 실행하는 방식으로 콘솔 창을 시작합니다.

### 파일 감시 도구 vs 툴체인 속도

백신과 기타 백그라운드 파일 감시 도구는 툴체인 설치 속도와 PX4 빌드 시간을 급격하게 줄일 수 있습니다.

빌드를 진행하는 동안에는 임시로 멈추는것이 좋겠습니다(대신 그동안에 일어나는 일은 여러분 책임입니다 :P).

### 윈도우와 Git의 개별 사례

#### Windows CR+LF vs Unix LF Line Endings

We recommend that you force Unix style LF endings for every repository you're working with using this toolchain (and use an editor which preserves them when saving your changes - e.g. Eclipse or VS Code). Compilation of source files also works with CR+LF endings checked out locally, but there are cases in Cygwin (e.g. execution of shell scripts) that require Unix line endings (otherwise you get errors like `$'\r': Command not found.`). Luckily git can do this for you when you execute the two commands in the root directory of your repo:

    git config core.autocrlf false
    git config core.eol lf
    

If you work with this toolchain on multiple repositories you can also set these two configurations globally for your machine:

    git config --global ...
    

This is not recommended because it may affect any other (unrelated) git use on your Windows machine.

#### Unix Permissions Execution Bit

Under Unix there's a flag in the permissions of each file that tells the OS whether or not the file is allowed to be executed. *git* under Cygwin supports and cares about that bit (even though the Windows NTFS file system does not use it). This often results in *git* finding "false-positive" differences in permissions. The resulting diff might look like this:

    diff --git ...
    old mode 100644
    new mode 100755
    

We recommend globally disabling the permission check on Windows to avoid the problem:

    git config --global core.fileMode false # disable execution bit check globally for the machine
    

For existing repositories that have this problem caused by a local configuration, additionally:

    git config --unset core.filemode # remove the local option for this repository to apply the global one
    git submodule foreach --recursive git config --unset core.filemode # remove the local option for all submodules
    

## 추가 정보

### 기능 / 문제 {#features}

The following features are known to work (version 2.0):

* Building and running SITL with jMAVSim with significantly better performance than a VM (it generates a native windows binary **px4.exe**).
* Building and uploading NuttX builds (e.g.: px4_fmu-v2 and px4_fmu-v4)
* Style check with *astyle* (supports the command: `make format`)
* Command line auto completion
* Non-invasive installer! The installer does NOT affect your system and global path (it only modifies the selected installation directory e.g. **C:\PX4** and uses a temporary local path).
* The installer supports updating to a new version keeping your personal changes inside the toolchain folder

생략:

* Simulation: Gazebo and ROS are not supported.
* Only NuttX and JMAVSim/SITL builds are supported.
* [Known problems](https://github.com/orgs/PX4/projects/6) (Also use to report issues).

### 셸 스크립트 설치 {#script_setup}

You can also install the environment using shell scripts in the Github project.

1. Make sure you have [Git for Windows](https://git-scm.com/download/win) installed.
2. Clone the repository https://github.com/PX4/windows-toolchain to the location you want to install the toolchain. Default location and naming is achieved by opening the `Git Bash` and executing:

    cd /c/
    git clone https://github.com/PX4/windows-toolchain PX4
    

1. If you want to install all components navigate to the freshly cloned folder and double click on the script `install-all-components.bat` located in the folder `toolchain`. If you only need certain components and want to safe Internet traffic and or disk space you can navigate to the different component folders like e.g. `toolchain\cygwin64` and click on the **install-XXX.bat** scripts to only fetch something specific.
2. Continue with [Getting Started](#getting_started) (or [Usage Instructions](#usage_instructions))

### 수동 설치 (툴체인 개발자용) {#manual_setup}

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

2. Add necessary **python packages** to your setup by opening the Cygwin toolchain console (double clicking **run-console.bat**) and executing
    
        pip2 install toml
        pip2 install pyserial
        pip2 install pyulog
        
    
    > **Note** That's what [cygwin64/install-cygwin-python-packages.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/cygwin64/install-cygwin-python-packages.bat) does.

3. Download the [**ARM GCC compiler**](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads) as zip archive of the binaries for Windows and unpack the content to the folder `C:\PX4\toolchain\gcc-arm`.
    
    > **Note** This is what the toolchain does in: [gcc-arm/install-gcc-arm.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/gcc-arm/install-gcc-arm.bat).

4. Install the JDK:
    
    * Download Java 14 from [Oracle](https://www.oracle.com/java/technologies/javase-jdk14-downloads.html) or [AdoptOpenJDK](https://adoptopenjdk.net/).
    * Because sadly there is no portable archive containing the binaries directly you have to install it.
    * Find the binaries and move/copy them to **C:\PX4\toolchain\jdk**.
    * You can uninstall the Kit from your Windows system again, we only needed the binaries for the toolchain.
    
    > **Note** This is what the toolchain does in: [jdk/install-jdk.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/jdk/install-jdk.bat).

5. 윈도우용 [**Apache Ant**](https://ant.apache.org/bindownload.cgi)를 zip 압축 파일 바이너리로 다운로드하고 `C:\PX4\toolchain\apache-ant` 폴더에 압축을 해제하십시오.
    
    > **Tip** Make sure you don't have an additional folder layer from the folder which is inside the downloaded archive.
    
    <span></span>
    
    > **Note** This is what the toolchain does in: [apache-ant/install-apache-ant.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/apache-ant/install-apache-ant.bat).

6. Download, build and add *genromfs* to the path:
    
    * Clone the source code to the folder **C:\PX4\toolchain\genromfs\genromfs-src** with 
            cd /c/toolchain/genromfs
            git clone https://github.com/chexum/genromfs.git genromfs-src

* 다음 명령으로 컴파일하십시오: 
    
        cd genromfs-src
         make all
    
    * Copy the resulting binary **genromfs.exe** one folder level out to: **C:\PX4\toolchain\genromfs**
    
    > **Note** This is what the toolchain does in: [genromfs/install-genromfs.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/genromfs/install-genromfs.bat).

1. Make sure all the binary folders of all the installed components are correctly listed in the `PATH` variable configured by [**setup-environment.bat**](https://github.com/PX4/windows-toolchain/blob/master/toolchain/scripts/setup-environment.bat).