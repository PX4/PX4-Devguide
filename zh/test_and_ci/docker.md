# PX4 Docker 容器

Docker 容器被提供用于完整的 [PX4 开发工具链](../setup/dev_env.md#supported-targets)，包括基于 NuttX 和 Linux 的硬件，[Gazebo Simulation](../simulation/gazebo.md) 和 [ROS](../simulation/ros_interface.md)。

本主题说明如何使用 [available docker containers](#px4_containers) 访问本地 Linux 计算机中的构建环境。

> **Note** Dockerfiles and README can be found on [Github here](https://github.com/PX4/containers/blob/master/README.md). 它们是在 [Docker Hub](https://hub.docker.com/u/px4io/) 上自动构建的。

## 系统必备组件

> **Note** PX4 容器目前仅在 Linux 上受支持（如果您没有 Linux，则可以在虚拟机内运行容器 [inside a virtual machine](#virtual_machine)）。 不要将 `boot2docker` 与默认的 Linux 映像一起使用，因为它不包含 X-Server。

为您的 Linux 计算机 [Install Docker](https://docs.docker.com/installation/)，最好使用 Docker 维护的一个软件包存储库来获取最新的稳定版本。 您可以使用 *Enterprise Edition* 或（free）*Community Edition*。

对于在 *Ubuntu* 上本地安装非生产设置，安装 Docker 的最快捷最简单的方法是使用 [convenience script](https://docs.docker.com/install/linux/docker-ce/ubuntu/#install-using-the-convenience-script)，如下所示（在同一页上找到替代安装方法）：

```sh
curl -fsSL get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```

默认安装要求您以 root 用户身份调用 *Docker*（用 `sudo`）。 然后，我们建议 [使用 docker 作为一个 non-root 用户](https://docs.docker.com/install/linux/linux-postinstall/#manage-docker-as-a-non-root-user)来构建PX4固件。 这样一来，在使用docker之后，你构建的文件夹将不会是归root所有。

```sh
＃创建 docker 组（可能不是必需的）
sudo groupadd docker
＃将您的用户添加到 docker 组。
sudo usermod -aG docker $ USER
＃在使用 docker 之前再次登录/注销！
```

## 本地编辑层次结构 {#px4_containers}

The available containers are listed below (from [Github](https://github.com/PX4/containers/blob/master/README.md#container-hierarchy)):

| 容器                              | 描述                             |
| ------------------------------- | ------------------------------ |
| px4-dev-base                    | 所有本地共有的基本设置                    |
| &emsp;px4-dev-nuttx             | NuttX 工具链                      |
| &emsp;px4-dev-simulation        | NuttX 工具链 + 仿真（jMAVSim，Gazebo） |
| &emsp;&emsp;px4-dev-ros         | NuttX 工具链，仿真 + ROS（包括 MAVROS）  |
| &emsp;px4-dev-raspi             | 树莓派工具链                         |
| &emsp;px4-dev-snapdragon        | 高通 Snapdragon Flight 工具链       |
| &emsp;px4-dev-clang             | C 语言工具                         |
| &emsp;&emsp;px4-dev-nuttx-clang | C 语言与 NuttX 工具                 |

The most recent version can be accessed using the `latest` tag: `px4io/px4-dev-nuttx:latest` (available tags are listed for each container on *hub.docker.com*. For example, the *px4-dev-ros* tags can be found [here](https://hub.docker.com/r/px4io/px4-dev-nuttx/tags)).

> **Tip** 通常，您应该使用最近的模式，但不一定是最新的模式（因为这经常更改）。

## 使用 Docker 容器

以下说明显示如何使用在 docker 容器中运行的工具链在主机上构建 PX4 源代码。 The information assumes that you have already downloaded the PX4 source code to **src/PX4-Autopilot**, as shown:

```sh
mkdir src
cd src
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
```

### 助手脚本（docker_run.sh）

The easiest way to use the containers is via the [docker_run.sh](https://github.com/PX4/PX4-Autopilot/blob/master/Tools/docker_run.sh) helper script. 此脚本将 PX4 构建命令作为参数（例如 `make tests`）。 它使用适当容器和合理环境设置的最新版本（硬编码）启动 docker。

For example, to build SITL you would call (from within the **/PX4-Autopilot** directory):

```sh
./Tools/docker_run.sh 'make px4_sitl_default'
```

或者使用 NuttX 工具链启动 bash 会话：

    ./Tools/docker_run.sh 'bash'
    

> **Tip** 脚本很简单，因为您不需要了解 *Docker* 或者考虑使用哪个容器。 但它不是特别准确！ 下面讨论的 [section below](#manual_start) 方法更灵活，如果您对脚本有任何问题，应该使用它。

### 手动调用 Docker {#manual_start}

典型命令的语法如下所示。 这将运行一个支持 X 指令的 Docker 容器（使容器内部的模拟 GUI 可用）。 它将目录 `&lt;host_src&gt;`from your computer to`&lt;container_src&gt;` 容器内，并转发连接 *QGroundControl* 所需的 UDP 端口。 使用 `-–privileged` 选项，它将自动访问主机上的设备（例如操纵杆和 GPU）。 如果连接/断开设备，则必须重新启动容器。

```sh
# enable access to xhost from the container
xhost +

# Run docker
docker run -it --privileged \
    --env=LOCAL_USER_ID="$(id -u)" \
    -v <host_src>:<container_src>:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e DISPLAY=:0 \
    -p 14570:14570/udp \
    --name=<local_container_name> <container>:<tag> <build_command>
```

位置：

* `&lt;host_src&gt;`：要映射到容器中的 `&lt;container_src&gt;` 的主计算机目录。 This should normally be the **PX4-Autopilot** directory.
* `&lt;container_src&gt;`：容器内的共享（源）目录的位置。
* `&lt;local_container_name&gt;`：正在创建的 docker 容器的名称 如果我们需要再次引用容器，以后可以使用它。
* `&lt;container&gt;：&lt;tag&gt;`：具有版本标签的容器 - 例如：`px4io/px4-dev-ros：2017-10-23`。
* `&lt;build_command&gt;`：要在新容器上调用的命令。 例如. `bash` 用于打开容器中的 bash shell。

The concrete example below shows how to open a bash shell and share the directory **~/src/PX4-Autopilot** on the host computer.

```sh
# enable access to xhost from the container
xhost +

# Run docker and open bash shell
docker run -it --privileged \
--env=LOCAL_USER_ID="$(id -u)" \
-v ~/src/PX4-Autopilot:/src/PX4-Autopilot/:rw \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-e DISPLAY=:0 \
-p 14570:14570/udp \
--name=mycontainer px4io/px4-dev-ros:2017-10-23 bash
```

如果一切顺利，你现在应该在一个新的 bash shell 中。 通过运行验证一切是否正常，例如，SITL：

```sh
cd src/PX4-Autopilot    #This is <container_src>
make px4_sitl_default gazebo
```

### 重新进入容器

`docker run` 命令只能用于创建新容器。 要重新进入此容器（将保留您的更改），只需执行以下操作：

```sh
# 启动 container
docker start container_name
# 在container中打开 bash shell
docker exec -it container_name bash
```

如果需要连接到容器的多个 shell，只需打开一个新 shell 并再次执行最后一个命令。

### 清理容器

有时您可能需要完全清除容器。 您可以使用其名称来执行此操作：

```sh
docker rm mycontainer
```

如果您忘记了名称，则可以列出非活动容器 Id，然后将其删除，如下所示：

```sh
docker ps -a -q
45eeb98f1dd9
docker rm 45eeb98f1dd9
```

### QGroundControl

运行模拟实例时，例如在 docker 容器内的 SITL 并通过 *QGroundControl* 从主机控制它，必须手动设置通信链接。 *QGroundControl* 的自动连接功能在此处不起作用。

在 *QGroundControl* 中，导航至 [Settings](https://docs.qgroundcontrol.com/en/SettingsView/SettingsView.html) 并选择“通信链接”。 创建使用 UDP 协议的新链接。 The port depends on the used [configuration](https://github.com/PX4/PX4-Autopilot/blob/master/ROMFS/px4fmu_common/init.d-posix/rcS) e.g. port 14570 for the SITL config. The IP address is the one of your docker container, usually 172.17.0.1/16 when using the default network. 可以使用以下命令找到Docker容器的IP地址（假设容器名称为` mycontainer </ 0>）：</p>

<pre><code class="sh">$ docker inspect -f '{ {range .NetworkSettings.Networks}}{ {.IPAddress}}{ {end}}' mycontainer
`</pre> 

> **Note** 上面的两个大括号之间不应存在空格（需要使用它们以避免gitbook中的UI渲染问题）。

### 故障处理

#### 权限错误

容器根据需要使用默认用户创建文件-通常为“root”。 这可能导致权限错误，其中主机上的用户无法访问容器创建的文件。

上面的示例使用行 `-env=LOCAL_USER_ID=“$（id-u）”` 在容器中创建具有与主机上的用户相同的 UID 的用户。 这可确保在主机上可以访问容器中创建的所有文件。

#### 图形驱动问题

运行 Gazebo 可能会导致类似以下错误消息：

```sh
libGL error: failed to load driver: swrast
```

在这种情况下，必须安装主机系统的本机图形驱动程序。 下载正确的驱动程序并将其安装在容器中。 对于 Nvidia 驱动程序，应使用以下命令（否则安装程序将从主机中看到已加载的模块并拒绝继续）：

```sh
./NVIDIA-DRIVER.run -a -N --ui=none --no-kernel-module
```

有关此内容的更多信息，请参见 [here](http://gernotklingler.com/blog/howto-get-hardware-accelerated-opengl-support-docker/)。

## 虚拟机支持 {#virtual_machine}

任何最新的 Linux 发行版应该可行。

测试以下配置：

* OS X 与 VMWare Fusion 和 Ubuntu 14.04（Parallels 上支持 GUI 的 Docker 容器使 X-Server 崩溃）。

**内存**

为虚拟机使用至少 4GB 内存。

**编译方案**

如果编译失败，则出现以下错误：

```sh
这个错误是不可复现的，可能是硬件或操作系统问题。
c++: internal compiler error: Killed (program cc1plus)
```

尝试禁用并行构建。

**允许从 VM 主机控制 Docker**

编辑 `/etc/defaults/docker` 并添加以下行：

```sh
DOCKER_OPTS="${DOCKER_OPTS} -H unix:///var/run/docker.sock -H 0.0.0.0:2375"
```

然后，您可以从主机操作系统控制 docker：

```sh
export DOCKER_HOST=tcp://<ip of your VM>:2375
# 运行一些 docker 命令检查是否正常工作，如：ps
docker ps
```