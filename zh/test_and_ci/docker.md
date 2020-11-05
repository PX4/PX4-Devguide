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

<a id="px4_containers"></a>

## Container Hierarchy

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

The following instructions show how to build PX4 source code on the host computer using a toolchain running in a docker container. The information assumes that you have already downloaded the PX4 source code to **src/PX4-Autopilot**, as shown:

```sh
mkdir src
cd src
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
```

### 助手脚本（docker_run.sh）

The easiest way to use the containers is via the [docker_run.sh](https://github.com/PX4/PX4-Autopilot/blob/master/Tools/docker_run.sh) helper script. This script takes a PX4 build command as an argument (e.g. `make tests`). It starts up docker with a recent version (hard coded) of the appropriate container and sensible environment settings.

For example, to build SITL you would call (from within the **/PX4-Autopilot** directory):

```sh
./Tools/docker_run.sh 'make px4_sitl_default'
```

Or to start a bash session using the NuttX toolchain:

    ./Tools/docker_run.sh 'bash'
    

> **Tip** 脚本很简单，因为您不需要了解 *Docker* 或者考虑使用哪个容器。 但它不是特别准确！ 下面讨论的 [section below](#manual_start) 方法更灵活，如果您对脚本有任何问题，应该使用它。

<a id="manual_start"></a>

### Calling Docker Manually

The syntax of a typical command is shown below. This runs a Docker container that has support for X forwarding (makes the simulation GUI available from inside the container). It maps the directory `<host_src>` from your computer to `<container_src>` inside the container and forwards the UDP port needed to connect *QGroundControl*. With the `-–privileged` option it will automatically have access to the devices on your host (e.g. a joystick and GPU). If you connect/disconnect a device you have to restart the container.

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

Where,

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

If everything went well you should be in a new bash shell now. Verify if everything works by running, for example, SITL:

```sh
cd src/PX4-Autopilot    #This is <container_src>
make px4_sitl_default gazebo
```

### 重新进入容器

The `docker run` command can only be used to create a new container. To get back into this container (which will retain your changes) simply do:

```sh
# 启动 container
docker start container_name
# 在container中打开 bash shell
docker exec -it container_name bash
```

If you need multiple shells connected to the container, just open a new shell and execute that last command again.

### 清理容器

Sometimes you may need to clear a container altogether. You can do so using its name:

```sh
docker rm mycontainer
```

If you can't remember the name, then you can list inactive container ids and then delete them, as shown below:

```sh
docker ps -a -q
45eeb98f1dd9
docker rm 45eeb98f1dd9
```

### QGroundControl

When running a simulation instance e.g. SITL inside the docker container and controlling it via *QGroundControl* from the host, the communication link has to be set up manually. The autoconnect feature of *QGroundControl* does not work here.

In *QGroundControl*, navigate to [Settings](https://docs.qgroundcontrol.com/en/SettingsView/SettingsView.html) and select Comm Links. Create a new link that uses the UDP protocol. The port depends on the used [configuration](https://github.com/PX4/PX4-Autopilot/blob/master/ROMFS/px4fmu_common/init.d-posix/rcS) e.g. port 14570 for the SITL config. The IP address is the one of your docker container, usually 172.17.0.1/16 when using the default network. The IP address of the docker container can be found with the following command (assuming the container name is `mycontainer`):

```sh
$ docker inspect -f '{ {range .NetworkSettings.Networks}}{ {.IPAddress}}{ {end}}' mycontainer
```

> **Note** 上面的两个大括号之间不应存在空格（需要使用它们以避免gitbook中的UI渲染问题）。

### 故障处理

#### 权限错误

The container creates files as needed with a default user - typically "root". This can lead to permission errors where the user on the host computer is not able to access files created by the container.

The example above uses the line `--env=LOCAL_USER_ID="$(id -u)"` to create a user in the container with the same UID as the user on the host. This ensures that all files created within the container will be accessible on the host.

#### 图形驱动问题

It's possible that running Gazebo will result in a similar error message like the following:

```sh
libGL error: failed to load driver: swrast
```

In that case the native graphics driver for your host system must be installed. Download the right driver and install it inside the container. For Nvidia drivers the following command should be used (otherwise the installer will see the loaded modules from the host and refuse to proceed):

```sh
./NVIDIA-DRIVER.run -a -N --ui=none --no-kernel-module
```

More information on this can be found [here](http://gernotklingler.com/blog/howto-get-hardware-accelerated-opengl-support-docker/).

<a id="virtual_machine"></a>

## Virtual Machine Support

Any recent Linux distribution should work.

The following configuration is tested:

* OS X 与 VMWare Fusion 和 Ubuntu 14.04（Parallels 上支持 GUI 的 Docker 容器使 X-Server 崩溃）。

**Memory**

Use at least 4GB memory for the virtual machine.

**Compilation problems**

If compilation fails with errors like this:

```sh
这个错误是不可复现的，可能是硬件或操作系统问题。
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
# 运行一些 docker 命令检查是否正常工作，如：ps
docker ps
```