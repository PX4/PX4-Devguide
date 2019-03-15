# 编译 PX4 软件

对于模拟目标和硬件目标，可以在控制台上或 IDE 中生成 PX4。

> **Note** 在执行这些说明之前，必须首先为主机操作系统和目标硬件安装 [开发者工具链](../setup/dev_env.md)。

## 下载 PX4 源代码 {#get_px4_code}

PX4 源代码存储在 [PX4/Firmware](https://github.com/PX4/Firmware) 存储库中的 GitHub 上。 我们建议您 [fork](https://help.github.com/articles/fork-a-repo/) 此存储库（创建与您自己的 Github 帐户关联的副本），然后将源 [clone](https://help.github.com/articles/cloning-a-repository/) 到本地计算机。

> **Tip** 通过从存储库建立分支，您可以更好地管理自定义代码。 稍后, 您将能够使用 *git* 与主项目共享更改。

建立分支及克隆项目源代码的步骤如下：

1. 在GitHub上 [注册](https://github.com/)。
2. 转到 [Firmware](https://github.com/PX4/Firmware) 存储库，然后单击右上角附近的 **Fork** 按钮。 这将创建并打开分叉存储库。
    
    ![Github 分支按钮](../../assets/toolchain/github_fork.png)

3. 复制 *Firmware* 存储库分叉的存储库URl。 最简单的方法是单击 **Clone 或下载** 按钮，然后复制URL：
    
    ![Github 克隆或下载按钮](../../assets/toolchain/github_clone_or_download.png)

4. 在计算机上打开命令提示终端
    
    - 在 OS X 上，点击 ⌘-space 并搜索 “terminal” 。 
    - 在 Ubuntu 上，单击运行栏并搜索 “terminal”。 
    - 在 Windows 上，在“开始”菜单中找到 PX4 文件夹，然后单击 "PX4 Console"。 

5. 使用复制的 URL 克隆存储库分支。 如下所示:
    
        git clone https://github.com/<youraccountname>/Firmware.git
        
    
    > **Tip** 如果您只是在尝试（并且不想进行任何永久性更改），则只需克隆主固件存储库，如下所示： ```git clone https://github.com/PX4/Firmware.git```
    
    Windows 用户 [参考 github 帮助 ](https://help.github.com/desktop/guides/getting-started-with-github-desktop/installing-github-desktop/)。 您可以使用 *git* 命令行客户端，如上所示，也可以使用 *Windows的Github * 应用程序执行相同的操作。

这将在计算机上复制 *大部分* *非常新的* PX4 源代码（在构建 PX4 时，代码的其余部分会自动从其他 [git 子模块](https://git-scm.com/book/en/v2/Git-Tools-Submodules) 中获取）。

<span id="specific_version_source"></span>

### 获取特定发行版本

要获取 *特定的旧发布* 的源代码：

1. 克隆固件存储库并导航到固件目录： 
        sh
        git clone https://github.com/PX4/Firmware.git
        cd Firmware

2. 列出所有发行版本（标签） 
        sh
        git tag -l

3. 迁出特定tag的代码（比如 tag为 1.7.4的beta版本） 
        sh
        git checkout v1.7.4beta

## 初次编译（使用 jMAVSim 模拟器） {#jmavsim_build}

初次编译之前，我们会使用终端环境编译一个模拟目标。 这使我们能够在进入真正的硬件和 IDE 之前验证系统设置。

导航到 **Firmware** 目录，并使用以下命令启动 [jMAVSim](../simulation/jmavsim.md)：

```sh
make px4_sitl jmavsim
```

这将在下面显示 PX4 控制台：

![PX4 控制台（jMAVSim）](../../assets/console_jmavsim.png)

无人机可以通过键入：

```sh
pxh> commander takeoff
```

![jMAVSim 界面](../../assets/jmavsim_first_takeoff.png)

无人机可以通过输入 `commander land` 着陆, 整个模拟可以通过 **CTRL+C**（或输入 `shutdown`）来停止。

与地面控制站一起飞行模拟更接近飞机的实际运行。 在飞机飞行时，单击地图上的某个位置（起飞飞行模式）并启用滑块。 这将重新定位飞机。

![QGroundControl 跳转](../../assets/qgc_goto.jpg)

> **Tip** PX4 可用于其他许多 [Simulators](../simulation/README.md)，包括 [Gazebo 模拟](../simulation/gazebo.md) 和 [AirSim 模拟](../simulation/airsim.md)。 这些也是从 *make* 开始的\----例如。 ```make px4_sitl gazebo```

## 基于NuttX / Pixhawk 的飞控板

### 构建 {#building_nuttx}

要构建基于 Nuttx 或 Pixhawk 的飞控板，请跳转到 **Firmware** 目录，然后调用 `make` 构建。

例如，要生成 *Pixracer* 您将使用以下命令：

```sh
cd Firmware
make px4_fmu-v4_default
```

> **Note** 在上面的生成目标的第一部分中 `px4_fmu-v4` 是特定飞行控制器硬件的固件，`默认` 是配置名称 （在本例中为 "默认" 配置）。 `默认` 是可选的，因此您可以改为执行以下操作： ```make px4_fmu-v4```

运行成功后将输出类似结束：

```sh
-- Build files have been written to: /home/youruser/src/Firmware/build/px4_fmu-v4_default
[954/954] Creating /home/youruser/src/Firmware/build/px4_fmu-v4_default/px4_fmu-v4_default.px4
```

下面的列表是常见飞控板的生成命令：

- Pixhawk 4: `make px4_fmu-v5_default`
- [Pixracer](https://docs.px4.io/en/flight_controller/pixracer.html): `make px4_fmu-v4_default`
- [Pixhawk 3 Pro](https://docs.px4.io/en/flight_controller/pixhawk3_pro.html): `make px4_fmu-v4pro_default`
- [Pixhawk Mini](https://docs.px4.io/en/flight_controller/pixhawk_mini.html): `make px4_fmu-v3_default`
- [Pixhawk 2](https://docs.px4.io/en/flight_controller/pixhawk-2.html): `make px4_fmu-v3_default`
- [mRo Pixhawk](https://docs.px4.io/en/flight_controller/mro_pixhawk.html): `make px4_fmu-v3_default`（支持 2MB 闪存）
- [HKPilot32](https://docs.px4.io/en/flight_controller/HKPilot32.html): `make px4_fmu-v2_default`
- [Pixfalcon](https://docs.px4.io/en/flight_controller/pixfalcon.html): `make px4_fmu-v2_default`
- [Dropix](https://docs.px4.io/en/flight_controller/dropix.html): `make px4_fmu-v2_default`
- [MindPX](https://docs.px4.io/en/flight_controller/mindpx.html)/[MindRacer](https://docs.px4.io/en/flight_controller/mindracer.html): `make airmind_mindpx-v2_default`
- [mRo X-2.1](https://docs.px4.io/en/flight_controller/mro_x2.1.html): `make auav_x21_default` 
- [Crazyflie 2.0](https://docs.px4.io/en/flight_controller/crazyflie2.html): `make bitcraze_crazyflie_default`
- [Intel® Aero Ready to Fly Drone](https://docs.px4.io/en/flight_controller/intel_aero.html): `make intel_aerofc-v1_default`
- [Pixhawk 1](https://docs.px4.io/en/flight_controller/pixhawk.html): `make px4_fmu-v2_default` > **Warning** 您 **必须** 使用 [版本支持的GCC](../setup/dev_env_linux_ubuntu.md#nuttx-based-hardware)编译（比如，和用于[CI/docker](../test_and_ci/docker.md)一样）或者从构建中删除模块。 使用不受支持的 GCC 构建可能会失败，因为 PX4 对飞控板有 1MB 的闪存限制。
- Pixhawk 1 的 2 MB 闪存版: `make px4_fmu-v3_default`

> **Note** 通常，`_default` 后缀是可选的（即，您也可以使用 `make px4_fmu-v4`、`make bitcraze_crazyflie` 等）生成。

### 将固件烧录到飞控板

附加 `upload` 到 make 命令，通过 USB 将编译的二进制文件烧录到自动驾驶仪硬件。 例如

```sh
make px4_fmu-v4_default upload
```

运行成功后将以如下结束：

```sh
Erase  : [====================] 100.0%
Program: [====================] 100.0%
Verify : [====================] 100.0%
Rebooting.

[100%] Built target upload
```

## 其他飞控板

下列飞控板有一些更复杂的构建和部署说明。

### Raspberry Pi 2/3

以下是 [Raspberry Pi 2/3 Navio2](https://docs.px4.io/en/flight_controller/raspberry_pi_navio2.html) 构建目标的命令。

#### 跨编译器生成

```sh
cd Firmware
make emlid_navio2_cross # for cross-compiler build
```

"PX4" 可执行文件位于目录 **build/emlid_navio2_cross/** 中。 请确保您可以通过 ssh 连接到 RPi，请参阅 [介绍如何访问您的 RPi](https://docs.px4.io/en/flight_controller/raspberry_pi_navio2.html#developer-quick-start)。

然后使用以下方法设置 RPi 的 IP（或主机名）。

```sh
export AUTOPILOT_HOST=192.168.X.X
```

并上传：

```sh
cd Firmware
make emlid_navio2_cross upload # for cross-compiler build
```

然后，通过 ssh 连接并运行（以 root 身份）：

```sh
sudo ./bin/px4 -s px4.config
```

#### 本机生成

如果要在 Pi 上生成 *directly*，则需要本机生成目标（emlid_navio2_native）。

```sh
cd Firmware
make emlid_navio2_native # for native build
```

"PX4" 可执行文件位于目录 **build/emlid_navio2_native/** 中。 直接运行:

```sh
sudo ./build/emlid_navio2_native/px4 -s ./posix-configs/rpi/px4.config
```

成功生成,，然后执行 PX4 将会看到如下内容：

```sh
<br />______  __   __    ___
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.


pxh&gt;
```

#### 自动启动

要自动启动 PX4，请将以下内容添加到文件 **/etc/rc.local**（如果使用本机生成，请相应地调整），在 `exit 0` 之前：

```sh
cd /home/pi && ./bin/px4 -d -s px4.config > px4.log
```

### Parrot Bebop

[Parrot Bebop](https://docs.px4.io/en/flight_controller/bebop.html)的支持还处于早期阶段，应当格外小心。

#### 编译

```sh
cd Firmware
make parrot_bebop_default
```

打开Bebop，使用WiFi与上位机连接。 然后，按四次电源键开启 ADB，启动 Telnet 进程。

```sh
make parrot_bebop_default upload
```

这将把 PX4 mainapp 上传到 /data/ftp/internal_000/ 并创建文件 /home/root/ 参数（如果尚未存在）。 这也将混频器文件和 px4.config 文件上传到 /homep/root/tol。

#### 运行

连接到 bebop 的 wifi，然后按下电源按钮四次。 接下来，通过 telnet 或 adb 外壳连接到 Bebop，并运行下面的命令。

```sh
telnet 192.168.42.1
```

使用命令杀死Bebop的专属驱动进程：

```sh
kk
```

启动 PX4 主机：

```sh
/data/ftp/internal_000/px4 -s /home/root/px4.config
```

要驾驶 Bebop，将操纵杆设备与主机连接，然后启动 QGroundControl。 Bebop 和操纵杆都应该被识别。 按照说明校准传感器并设置操纵杆设备。

#### 自动启动

要在启动 Bebop 上自动启动 PX4，请修改 init 脚本 `/etc/init.d/rcS_mode_default`。 请对以下行进行注释：

    DragonStarter.sh -out2null &
    

替换为：

    echo 1 > /sys/class/gpio/gpio85/value # enables the fan
    /data/ftp/internal_000/px4 -d -s /home/root/px4.config > /home/root/px4.log &
    

按四次电源键开启 adb 服务，用如下方式连接到 adb 服务：

```sh
adb connect 192.168.42.1:9050
```

把系统分区重新挂载成可读可写：

```sh
adb shell mount -o remount,rw /
```

为了避免手动编辑文件，可以直接使用如下文件： https://gist.github.com/bartslinger/8908ff07381f6ea3b06c1049c62df44e

备份原始文件，并将下载的文件传到 Bebop

```sh
adb shell cp /etc/init.d/rcS_mode_default /etc/init.d/rcS_mode_default_backup
adb push rcS_mode_default /etc/init.d/
```

同步后重启：

```sh
adb shell sync
adb shell reboot
```

### OcPoC-Zynq Mini

[OcPoC-Zynq Mini](https://docs.px4.io/en/flight_controller/ocpoc_zynq.html) 的编译说明参见：

- [Aerotenna OcPoC-Zynq Mini Flight Controller > Building PX4 for OcPoC-Zynq](https://docs.px4.io/en/flight_controller/ocpoc_zynq.html#building-px4-for-ocpoc-zynq)（PX4 用户手册）
- [OcPoC PX4 构建页](https://aerotenna.readme.io/docs/px4-setup)

### 基于 QuRT / Snapdragon 的飞控板

该部分介绍 [高通骁龙飞控](https://docs.px4.io/en/flight_controller/snapdragon_flight.html) 如何编译：

#### 编译

> **Note** 如果您使用的是 [高通电调主板](http://shop.intrinsyc.com/products/qualcomm-electronic-speed-control-board)（基于串口），请移步 [这里](https://github.com/ATLFlight/ATLFlightDocs/blob/master/PX4.md)。 如果您使用正常的基于 PWM 的电调板，则可以继续按照此页上的说明进行操作。

下面的命令构建了 Linux 和 DSP 端的目标。 两个可执行文件都通过 [muORB](../middleware/uorb.md) 进行通信。

```sh
cd Firmware
make atlflight_eagle_default
```

要在设备上加载 SW，请通过 usb 连接线进行连接，并确保设备已启动。 在新的终端窗口中运行此操作：

```sh
adb shell
```

到上一个终端并上传：

```sh
make atlflight_eagle_default upload
```

请注意，这也将覆盖两个配置文件 [mainapp.config](https://github.com/PX4/Firmware/blob/master/posix-configs/eagle/flight/mainapp.config) 并 [px4.config](https://github.com/PX4/Firmware/blob/master/posix-configs/eagle/flight/px4.config) 到设备。 如果你想编辑飞机的启动脚本，这些文件分别保存在 /usr/share/data/adsp/px4.config 和 /home/linaro/mainapp.config。

混频器现在需要手动复制:

```sh
adb push ROMFS/px4fmu_common/mixers/quad_x.main.mix  /usr/share/data/adsp
```

#### 运行脚本

运行 DSP 调试监控器：

```sh
${HEXAGON_SDK_ROOT}/tools/debug/mini-dm/Linux_Debug/mini-dm
```

注意：在 Mac 上可以使用 [nano-dm](https://github.com/kevinmehall/nano-dm)。

继续使用 ADB shell 运行 PX4：

```sh
cd /home/linaro
./px4 -s mainapp.config
```

请注意, 断开 USB 后，PX4 将立即停止（或者如果您的 ssh 会话已断开连接）。 要飞行，您应该使 PX4 上电后自动启动。

#### 自动启动

要在骁龙启动后立即运行 PX4，您可以将启动添加到 `rc.local`：

请直接在骁龙上编辑文件 `/etc/rc.local`：

```sh
adb shell
vim /etc/rc.local
```

或者将文件复制到计算机，在本地编辑，然后将其复制回：

```sh
adb pull /etc/rc.local
gedit rc.local
adb push rc.local /etc/rc.local
```

对于自动启动，请在 `exit 0 ` 之前添加以下行：

```sh
(cd /home/linaro && ./px4 -s mainapp.config > mainapp.log)

exit 0
```

确保 `rc.local` 是可执行的：

```sh
adb shell
chmod +x /etc/rc.local
```

然后重新启动骁龙：

```sh
adb reboot
```

## 用图形界面 IDE 编译

PX4 支持 Qt Creator，Eclipse 和 Sublime Text。 Qt Creator 是最用户友好的，因此是唯一官方支持的 IDE。 除非是 Eclipse 或者 Sublime Text 的专家，否则不推荐。 硬核玩家会在源码目录里找到 [Eclipse project](https://github.com/PX4/Firmware/blob/master/eclipse.project) 和 [Sublime project](https://github.com/PX4/Firmware/blob/master/Firmware.sublime-project)。

{% youtube %}https://www.youtube.com/watch?v=Bkk8zttWxEI&rel=0&vq=hd720{% endyoutube %}

## Qt Creator 功能

Qt creator 提供符号跳转、自动补全和编译固件的功能。

![](../../assets/toolchain/qtcreator.png)

### 在 Linux 上使用 Qt creator

开启 Qt creator 之前，需要新建 [项目文件](https://cmake.org/Wiki/CMake_Generator_Specific_Information#Code::Blocks_Generator)。

```sh
cd ~/src/Firmware
mkdir ../Firmware-build
cd ../Firmware-build
cmake ../Firmware -G "CodeBlocks - Unix Makefiles"
```

然后通过 **文件 > 打开项目** 加载根目录下的 CMakeLists.txt。

加载后，可以将 **play** 按钮配置为运行目标配置中选择 "自定义可执行文件"，并输入 "make" 作为执行命令， "upload" 作为参数。

### 在 Windows 上使用 Qt creator

> **Note** Windows 平台下尚未测试。

### 在 Mac OS 上使用 Qt creator

开启 Qt creator 之前，需要新建 [项目文件](https://cmake.org/Wiki/CMake_Generator_Specific_Information#Code::Blocks_Generator)。

```sh
cd ~/src/Firmware
mkdir -p build/creator
cd build/creator
cmake ../.. -G "CodeBlocks - Unix Makefiles"
```

设置完成！ 启动 *Qt Creator</0 >，然后完成下面视频中的步骤，以设置要生成的项目。</p> 

{% youtube %}https://www.youtube.com/watch?v=0pa0gS30zNw&rel=0&vq=hd720{% endyoutube %}

## PX4 创建生成目标 {#make_targets}

前面的部分演示了如何调用 *make* 来构建多个不同的目标、启动模拟器、使用 IDE 等。 本节介绍如何构造 *make* 选项以及如何查找可用选项。

使用特定的配置和初始化文件调用 *make* 的完整语法是：

```sh
make [VENDOR_][MODEL][_VARIANT] [VIEWER_MODEL_DEBUGGER]
```

**VENDOR_MODEL_VARIANT**: (也称为 `CONFIGURATION_TARGET`)

- **VENDOR:** 飞控板制造商：`px4`，`aerotenna`，`airmind`，`atlflight`，`auav`，`beaglebone`，`intel`，`nxp`，`parrot`等。 Pixhawk 系列飞控板对应 `PX4`。
- **MODEL：** *飞控板模型</1 >"模型 "：`sitl`、`fmu-v2`、`fmu-v3`、`fmu-v4`、`fmu-v5`、`navio2` 等。</li> 
    
    - **VARIANT:**特定配置：例如 `rtps`、`lpe`，其中包含 `默认` 配置中不存在的组件。 最常见的是 `default`，可以省略。</ul> 
    
    > **Tip** 您可以使用下面的命令获取 *所有* 可用的 `CONFIGURATION_TARGET` 选项的列表： 
    > 
    >     sh
    >       make list_config_targets
    
    **VIEWER_MODEL_DEBUGGER:**
    
    - **VIEWER:**这是启动和连接的模拟器（"查看器"）：`gazebo`, `jmavsim` <!-- , ?airsim -->
    
    - **MODEL:**要使用的 *载具* 模型（例如 `iris` (*default*)、`rover`、`tailsitter` 等），该模型将由模拟器加载。 环境变量 `PX4_SIM_MODEL` 将设置为所选模型。然后在 [启动脚本 ](#scripts) 中使用该模型来选择适当的参数。
    
    - **DEBUGGER:**要使用的调试器：`none` (*default*)、`ide`、`gdb`、`lldb`、`ddd`、`valgrind`、`callgrind`。 有关详细信息，请参阅 < 0>Simulation 调试 </0 >。
    
    > **Tip** 您可以使用下面的命令获取 *所有* 可用的 `VIEWER_MODEL_DEBUGGER` 选项的列表： 
    > 
    >     sh
    >       make px4_sitl list_vmd_make_targets
    
    备注：
    
    - `CONFIGURATION_TARGET` 和 `VIEWER_MODEL_DEBUGGER` 中的大多数值都有默认值, 因此是可选的。 比如，`gazebo` 相当于 `gazebo_iris` 或 `gazebo_iris_none` 。 
    - 如果要在其他两个设置之间指定默认值，可以使用三个下划线。 例如，`gazebo___gdb` 等效于 `gazebo_iris_gdb`。
    - 您可以使用 `VIEWER_MODEL_DEBUGGER` 的 `none` 值启动 PX4 并等待模拟器。 For example start PX4 using `make px4_sitl_default none` and jMAVSim using `./Tools/jmavsim_run.sh -l`.
    
    `VENDOR_MODEL_VARIANT` 选项映射到 [/boards](https://github.com/PX4/Firmware/tree/master/boards) 目录下的 PX4 特定的 *cmake* 配置文件。 具体而言 `VENDOR_MODEL_VARIANT` 映射到配置文件 **boards/VENDOR/MODEL/VARIANT.cmake**（例如 `px4_fmu-v5_default` 对应于 [boards/px4/fmu-v5/default.cmake](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v5/default.cmake)）。
    
    以下各节讨论了其他目标生成的问题（包括但不限于）：
    
    ### 二进制文件大小剖析 {#bloaty_compare_master}
    
    `bloaty_compare_master` 构建目标使您能够更好地了解更改对代码大小的影响。当使用时，工具链会下载特定固件的最新的 master 版本并将其与本地生成进行比较（使用二进制文件的 [bloaty](https://github.com/google/bloaty) 大小探查器）。
    
    > **Tip** 这有助于分析（可能）导致 `px4_fmu-v2_default` 达到1MB 闪存限制的更改。
    
    *Bloaty* 必须在您的路径中，并且在 *cmake* 配置时找到。 PX4 [docker 文件 ](https://github.com/PX4/containers/blob/master/docker/px4-dev/Dockerfile_nuttx) 安装 *bloaty* 如下所示：
    
        git clone --recursive https://github.com/google/bloaty.git /tmp/bloaty \
            && cd /tmp/bloaty && cmake -GNinja . && ninja bloaty && cp bloaty /usr/local/bin/ \
            && rm -rf /tmp/*
        
    
    下面的示例演示如何查看从 `px4_fmu-v2_default` 中删除 *mpu9250* 驱动程序的影响。 首先，它在本地构建一个没有驱动程序的生成：
    
    ```sh
     % git diff
    diff --git a/boards/px4/fmu-v2/default.cmake b/boards/px4/fmu-v2/default.cmake
    index 40d7778..2ce7972 100644
    --- a/boards/px4/fmu-v2/default.cmake
    +++ b/boards/px4/fmu-v2/default.cmake
    @@ -36,7 +36,7 @@ px4_add_board(
                    imu/l3gd20
                    imu/lsm303d
                    imu/mpu6000
    
    -               imu/mpu9250
    +               #imu/mpu9250
                    #iridiumsbd
                    #irlock
                    #magnetometer # all available magnetometer drivers
    ```
    
    然后使用 make ，编译一个特殊版本进行比较（本例中为 `px4_fmu-v2_default`）：
    
    ```sh
    % make px4_fmu-v2_default bloaty_compare_master
    ...
    ...
    ...
         VM SIZE                                                                                        FILE SIZE
     --------------                                                                                  --------------
      [DEL]     -52 MPU9250::check_null_data(unsigned int*, unsigned char)                               -52  [DEL]
      [DEL]     -52 MPU9250::test_error()                                                                -52  [DEL]
      [DEL]     -52 MPU9250_gyro::MPU9250_gyro(MPU9250*, char const*)                                    -52  [DEL]
      [DEL]     -56 mpu9250::info(MPU9250_BUS)                                                           -56  [DEL]
      [DEL]     -56 mpu9250::regdump(MPU9250_BUS)                                                        -56  [DEL]
    ...                                        -336  [DEL]
      [DEL]    -344 MPU9250_mag::_measure(ak8963_regs)                                                  -344  [DEL]
      [DEL]    -684 MPU9250::MPU9250(device::Device*, device::Device*, char const*, char const*, cha    -684  [DEL]
      [DEL]    -684 MPU9250::init()                                                                     -684  [DEL]
      [DEL]   -1000 MPU9250::measure()                                                                 -1000  [DEL]
     -41.3%   -1011 [43 Others]                                                                        -1011 -41.3%
      -1.0% -1.05Ki [Unmapped]                                                                       +24.2Ki  +0.2%
      -1.0% -10.3Ki TOTAL                                                                            +14.9Ki  +0.1%
    ```
    
    可以看出，从 `px4_fmu-v2_default` 删除 *mpu9250* 驱动，可以节省 10.3KB 的 flash 空间。 它还显示了 *mpu9250* 驱动程序的不同部件的大小。