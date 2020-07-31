# PX4 도커 컨테이너

도커 컨테이너는 NuttX와 리눅스 기반 하드웨어 [가제보 모의시험](../simulation/gazebo.md), [ROS](../simulation/ros_interface.md)가 들어있는 완전한 [PX4 개발 툴체인](../setup/dev_env.md#supported-targets)을 제공합니다.

이 주제에서는 로컬 리눅스 컴퓨터에서 빌드 환경에 접근할 수 있는 [가용 도커 컨테이너](#px4_containers) 활용법을 알려드리도록 하겠습니다.

> **Note** Dockerfile 과 README 는 [이 곳 Github](https://github.com/PX4/containers/blob/master/README.md)에 있습니다. 이 파일은 [도커 허브](https://hub.docker.com/u/px4io/)에 자동으로 만들어줍니다.

## 준비 요건

> **Note** PX4 컨테이너는 현재 리눅스만 지원합니다(리눅스를 설치하지 않았다면 [가상 머신에서](#virtual_machine) 컨테이너를 실행할 수 있습니다). X 서버가 들어있지 않으므로 기본 리눅스 이미지에 대해 `boot2docker`를 실행하지 마십시오

리눅스 컴퓨터에 [도커를 설치하십시오](https://docs.docker.com/installation/). 도커 사이트에서 관리하는 꾸러미 저장소에서 적당한 최신 안정 꾸러미 하나를 활용하십시오. *기업용판* 또는 (무료) *커뮤니티판*을 활용할 수 있습니다.

For local installation of non-production setups on *Ubuntu*, the quickest and easiest way to install Docker is to use the [convenience script](https://docs.docker.com/install/linux/docker-ce/ubuntu/#install-using-the-convenience-script) as shown below (alternative installation methods are found on the same page):

```sh
curl -fsSL get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```

The default installation requires that you invoke *Docker* as the root user (i.e. using `sudo`). However, for building the PX4 firwmare we suggest to [use docker as a non-root user](https://docs.docker.com/install/linux/linux-postinstall/#manage-docker-as-a-non-root-user). That way, your build folder won't be owned by root after using docker.

```sh
# Create docker group (may not be required)
sudo groupadd docker
# Add your user to the docker group.
sudo usermod -aG docker $USER
# Log in/out again before using docker!
```

## Container Hierarchy {#px4_containers}

The available containers are listed below (from [Github](https://github.com/PX4/containers/blob/master/README.md#container-hierarchy)):

| Container                       | Description                                      |
| ------------------------------- | ------------------------------------------------ |
| px4-dev-base                    | Base setup common to all containers              |
| &emsp;px4-dev-nuttx             | NuttX toolchain                                  |
| &emsp;px4-dev-simulation        | NuttX toolchain + simulation (jMAVSim, Gazebo)   |
| &emsp;&emsp;px4-dev-ros         | NuttX toolchain, simulation + ROS (incl. MAVROS) |
| &emsp;px4-dev-raspi             | Raspberry Pi toolchain                           |
| &emsp;px4-dev-snapdragon        | Qualcomm Snapdragon Flight toolchain             |
| &emsp;px4-dev-clang             | Clang tools                                      |
| &emsp;&emsp;px4-dev-nuttx-clang | Clang and NuttX tools                            |

The most recent version can be accessed using the `latest` tag: `px4io/px4-dev-nuttx:latest` (available tags are listed for each container on *hub.docker.com*. For example, the *px4-dev-ros* tags can be found [here](https://hub.docker.com/r/px4io/px4-dev-nuttx/tags)).

> **Tip** Typically you should use a recent container, but not necessarily the latest (as this changes too often).

## Use the Docker Container

The following instructions show how to build PX4 source code on the host computer using a toolchain running in a docker container. The information assumes that you have already downloaded the PX4 source code to **src/Firmware**, as shown:

```sh
mkdir src
cd src
git clone https://github.com/PX4/Firmware.git
cd Firmware
```

### Helper Script (docker_run.sh)

The easiest way to use the containers is via the [docker_run.sh](https://github.com/PX4/Firmware/blob/master/Tools/docker_run.sh) helper script. This script takes a PX4 build command as an argument (e.g. `make tests`). It starts up docker with a recent version (hard coded) of the appropriate container and sensible environment settings.

For example, to build SITL you would call (from within the **/Firmware** directory):

```sh
./Tools/docker_run.sh 'make px4_sitl_default'
```

Or to start a bash session using the NuttX toolchain:

    ./Tools/docker_run.sh 'bash'
    

> **Tip** The script is easy because you don't need to know anything much about *Docker* or think about what container to use. However it is not particularly robust! The manual approach discussed in the [section below](#manual_start) is more flexible and should be used if you have any problems with the script.

### Calling Docker Manually {#manual_start}

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

* `<host_src>`: The host computer directory to be mapped to `<container_src>` in the container. This should normally be the **Firmware** directory.
* `<container_src>`: The location of the shared (source) directory when inside the container.
* `<local_container_name>`: A name for the docker container being created. This can later be used if we need to reference the container again.
* `<container>:<tag>`: The container with version tag to start - e.g.: `px4io/px4-dev-ros:2017-10-23`.
* `<build_command>`: The command to invoke on the new container. E.g. `bash` is used to open a bash shell in the container.

The concrete example below shows how to open a bash shell and share the directory **~/src/Firmware** on the host computer.

```sh
# enable access to xhost from the container
xhost +

# Run docker and open bash shell
docker run -it --privileged \
--env=LOCAL_USER_ID="$(id -u)" \
-v ~/src/Firmware:/src/firmware/:rw \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-e DISPLAY=:0 \
-p 14570:14570/udp \
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
docker start container_name
# open a new bash shell in this container
docker exec -it container_name bash
```

If you need multiple shells connected to the container, just open a new shell and execute that last command again.

### Clearing the Container

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

In *QGroundControl*, navigate to [Settings](https://docs.qgroundcontrol.com/en/SettingsView/SettingsView.html) and select Comm Links. Create a new link that uses the UDP protocol. The port depends on the used [configuration](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d-posix/rcS) e.g. port 14570 for the SITL config. The IP address is the one of your docker container, usually 172.17.0.1/16 when using the default network. The IP address of the docker container can be found with the following command (assuming the container name is `mycontainer`):

```sh
$ docker inspect -f '{ {range .NetworkSettings.Networks}}{ {.IPAddress}}{ {end}}' mycontainer
```

> **Note** Spaces between double curly braces above should be not be present (they are needed to avoid a UI rendering problem in gitbook).

### Troubleshooting

#### Permission Errors

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

## Virtual Machine Support {#virtual_machine}

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