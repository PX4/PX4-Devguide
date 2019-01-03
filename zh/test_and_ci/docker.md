# PX4 Docker 容器

Docker 容器被提供用于完整的 [PX4 开发工具链](../setup/dev_env.md#supported-targets)，包括基于 NuttX 和 Linux 的硬件，[Gazebo Simulation](../simulation/gazebo.md) 和 [ROS](../simulation/ros_interface.md)。

本主题说明如何使用 [available docker containers](#px4_containers) 访问本地 Linux 计算机中的构建环境。

> **Note** Dockerfiles 和 README 可以在 [Github here](https://github.com/PX4/containers/tree/master/docker/px4-dev)。 它们是在 [Docker Hub](https://hub.docker.com/u/px4io/) 上自动构建的。

## 系统必备组件

> **Note** PX4 容器目前仅在 Linux 上受支持（如果您没有 Linux，则可以在虚拟机内运行容器 [inside a virtual machine](#virtual_machine)）。 不要将 `boot2docker` 与默认的 Linux 映像一起使用，因为它不包含 X-Server。

为您的 Linux 计算机 [Install Docker](https://docs.docker.com/installation/)，最好使用 Docker 维护的一个软件包存储库来获取最新的稳定版本。 您可以使用 *Enterprise Edition* 或（free）*Community Edition*。

对于在 *Ubuntu* 上本地安装非生产设置，安装 Docker 的最快捷最简单的方法是使用 [convenience script](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/#install-using-the-convenience-script)，如下所示（在同一页上找到替代安装方法）：

```sh
curl -fsSL get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```

默认安装要求您以 root 用户身份调用 * Docker*（即使用` sudo `）。 如果您希望 [use Docker as a non-root user](https://docs.docker.com/engine/installation/linux/linux-postinstall/#manage-docker-as-a-non-root-user)，您可以选择将用户添加到“docker”组，然后注销或者登陆：

```sh
＃创建 docker 组（可能不是必需的）
sudo groupadd docker
＃将您的用户添加到 docker 组。
sudo usermod -aG docker $ USER
＃在使用 docker 之前再次登录/注销！
```

## 本地编辑层次结构 {#px4_containers}

下面列出了可用的本地编辑（来自 [Github](https://github.com/PX4/containers/blob/master/docker/px4-dev/README.md#container-hierarchy)）：

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

可以使用 `latest` 标记访问最新版本：`px4io/px4-dev-ros:latest`（为 *hub.docker.com* 上的每个容器列出可用标记。 例如，*px4-dev-ros* 标签可以在 [here](https://hub.docker.com/r/px4io/px4-dev-ros/tags/)）。

> **Tip** 通常，您应该使用最近的模式，但不一定是最新的模式（因为这经常更改）。

## 使用 Docker 容器

以下说明显示如何使用在 docker 容器中运行的工具链在主机上构建 PX4 源代码。 该信息假定您已将 PX4 源代码下载到 **src/Firmware**，如下所示：

```sh
mkdir src
cd src
git clone https://github.com/PX4/Firmware.git
cd Firmware
```

### 助手脚本（docker_run.sh）

使用容器的最简单方法是通过 [docker_run.sh](https://github.com/PX4/Firmware/blob/master/Tools/docker_run.sh) 帮助程序脚本。 此脚本将 PX4 构建命令作为参数（例如 `make tests`）。 它使用适当容器和合理环境设置的最新版本（硬编码）启动 docker。

例如，要构建 SITL，您将调用（从 **/Firmware** 目录中）：

```sh
sudo ./Tools/docker_run.sh 'make px4_sitl_default'
```

或者使用 NuttX 工具链启动 bash 会话：

    sudo ./Tools/docker_run.sh 'bash'
    

> **Tip** 脚本很简单，因为您不需要了解 *Docker* 或者考虑使用哪个容器。 但它不是特别准确！ 下面讨论的 [section below](#manual_start) 方法更灵活，如果您对脚本有任何问题，应该使用它。

### 手动调用 Docker {#manual_start}

典型命令的语法如下所示。 这将运行一个支持 X 指令的 Docker 容器（使容器内部的模拟 GUI 可用）。 它将目录 `&lt;host_src&gt;`from your computer to`&lt;container_src&gt;` 容器内，并转发连接 *QGroundControl* 所需的 UDP 端口。 使用 `-–privileged` 选项，它将自动访问主机上的设备（例如操纵杆和 GPU）。 如果连接/断开设备，则必须重新启动容器。

```sh
# enable access to xhost from the container
xhost +

# Run docker
docker run -it --privileged \
    --env=LOCAL_USER_ID="$(id -u)" \
    -v &lt;host_src&gt;:&lt;container_src&gt;:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e DISPLAY=:0 \
    -p 14556:14556/udp \
    --name=&lt;local_container_name&gt; &lt;container&gt;:&lt;tag&gt; &lt;build_command&gt;
```

位置：

* `&lt;host_src&gt;`：要映射到容器中的 `&lt;container_src&gt;` 的主计算机目录。 这通常应该是 **Firmware** 目录。
* `&lt;container_src&gt;`：容器内的共享（源）目录的位置。
* `&lt;local_container_name&gt;`：正在创建的 docker 容器的名称 如果我们需要再次引用容器，以后可以使用它。
* `&lt;container&gt;：&lt;tag&gt;`：具有版本标签的容器 - 例如：`px4io/px4-dev-ros：2017-10-23`。
* `&lt;build_command&gt;`：要在新容器上调用的命令。 例如. `bash` 用于打开容器中的 bash shell。

下面的具体示例显示了如何打开 bash shell 并在主机上共享目录 **〜/src/Firmware**。

```sh
# enable access to xhost from the container
xhost +

# Run docker and open bash shell
sudo docker run -it --privileged \
--env=LOCAL_USER_ID="$(id -u)" \
-v ~/src/Firmware:/src/firmware/:rw \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-e DISPLAY=:0 \
-p 14556:14556/udp \
--name=mycontainer px4io/px4-dev-ros:2017-10-23 bash
```

If everything went well you should be in a new bash shell now. Verify if everything works by running, for example, SITL:

```sh
cd src/firmware    #This is <container_src>
make px4_sitl_default gazebo
```

### Re-enter the Container

The `docker run` command can only be used to create a new container. To get back into this container (which will retain your changes) simply do:

```sh
# start the container
sudo docker start container_name
# open a new bash shell in this container
sudo docker exec -it container_name bash
```

If you need multiple shells connected to the container, just open a new shell and execute that last command again.

### Clearing the Container

Sometimes you may need to clear a container altogether. You can do so using its name:

```sh
$ sudo docker rm mycontainer
```

If you can't remember the name, then you can list inactive container ids and then delete them, as shown below:

```sh
$ sudo docker ps -a -q
45eeb98f1dd9
$ sudo docker rm 45eeb98f1dd9
```

### QGroundControl

When running a simulation instance e.g. SITL inside the docker container and controlling it via *QGroundControl* from the host, the communication link has to be set up manually. The autoconnect feature of *QGroundControl* does not work here.

In *QGroundControl*, navigate to [Settings](https://docs.qgroundcontrol.com/en/SettingsView/SettingsView.html) and select Comm Links. Create a new link that uses the UDP protocol. The port depends on the used [configuration](https://github.com/PX4/Firmware/tree/master/posix-configs/SITL) e.g. port 14557 for the SITL iris config. The IP address is the one of your docker container, usually 172.17.0.1/16 when using the default network.

### 故障处理

#### 权限错误

The container creates files as needed with a default user - typically "root". This can lead to permission errors where the user on the host computer is not able to access files created by the container.

The example above uses the line `--env=LOCAL_USER_ID="$(id -u)"` to create a user in the container with the same UID as the user on the host. This ensures that all files created within the container will be accessible on the host.

#### Graphics Driver Issues

It's possible that running Gazebo will result in a similar error message like the following:

```sh
libGL error: failed to load driver: swrast
```

In that case the native graphics driver for your host system must be installed. Download the right driver and install it inside the container. For Nvidia drivers the following command should be used (otherwise the installer will see the loaded modules from the host and refuse to proceed):

```sh
./NVIDIA-DRIVER.run -a -N --ui=none --no-kernel-module
```

More information on this can be found [here](http://gernotklingler.com/blog/howto-get-hardware-accelerated-opengl-support-docker/).

## 虚拟机支持 {#virtual_machine}

Any recent Linux distribution should work.

The following configuration is tested:

* OS X with VMWare Fusion and Ubuntu 14.04 (Docker container with GUI support on Parallels make the X-Server crash).

**Memory**

Use at least 4GB memory for the virtual machine.

**Compilation problems**

If compilation fails with errors like this:

```sh
The bug is not reproducible, so it is likely a hardware or OS problem.
c++: internal compiler error: Killed (program cc1plus)
```

Try disabling parallel builds.

**Allow Docker Control from the VM Host**

Edit `/etc/defaults/docker` and add this line:

```sh
DOCKER_OPTS="${DOCKER_OPTS} -H unix:///var/run/docker.sock -H 0.0.0.0:2375"
```

You can then control docker from your host OS:

```sh
export DOCKER_HOST=tcp://<ip of your VM>:2375
# run some docker command to see if it works, e.g. ps
docker ps
```

## 旧版

The ROS multiplatform containers are not maintained anymore: https://github.com/PX4/containers/tree/master/docker/ros-indigo