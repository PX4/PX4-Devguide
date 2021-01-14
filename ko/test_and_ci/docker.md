!REDIRECT "https://docs.px4.io/master/ko/test_and_ci/docker.html"

# PX4 도커 컨테이너

도커 컨테이너는 NuttX와 리눅스 기반 하드웨어 [가제보 모의시험](../simulation/gazebo.md), [ROS](../simulation/ros_interface.md)가 들어있는 완전한 [PX4 개발 툴체인](../setup/dev_env.md#supported-targets)을 제공합니다.

이 주제에서는 로컬 리눅스 컴퓨터에서 빌드 환경에 접근할 수 있는 [가용 도커 컨테이너](#px4_containers) 활용법을 알려드리도록 하겠습니다.

> **Note** Dockerfile 과 README 는 [이 곳 Github](https://github.com/PX4/containers/blob/master/README.md)에 있습니다. 이 파일은 [도커 허브](https://hub.docker.com/u/px4io/)에 자동으로 만들어줍니다.

## 준비 요건

> **Note** PX4 컨테이너는 현재 리눅스만 지원합니다(리눅스를 설치하지 않았다면 [가상 머신에서](#virtual_machine) 컨테이너를 실행할 수 있습니다). X 서버가 들어있지 않으므로 기본 리눅스 이미지에 대해 `boot2docker`를 실행하지 마십시오

리눅스 컴퓨터에 [도커를 설치하십시오](https://docs.docker.com/installation/). 도커 사이트에서 관리하는 꾸러미 저장소에서 적당한 최신 안정 꾸러미 하나를 활용하십시오. *기업용판* 또는 (무료) *커뮤니티판*을 활용할 수 있습니다.

*우분투*에서 비 프로덕션 설정 방식으로 로컬에 설치하려면, 아래에 보여드리는 바와 같이 [간편 스크립트](https://docs.docker.com/install/linux/docker-ce/ubuntu/#install-using-the-convenience-script)를 활용하여 도커를 설치하는 방법이 가장 빠르고 간단한 방법입니다(대안 설치 방식도 동일한 페이지에 있습니다):

```sh
curl -fsSL get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```

기본 설치시 *도커*를 루트 사용자로 실행해야 합니다(예: `sudo` 활용). 그러나 PX4 펌웨어를 빌드하려면 [비 루트 사용자 계정으로 도커를 실행](https://docs.docker.com/install/linux/linux-postinstall/#manage-docker-as-a-non-root-user) 하시는게 좋습니다. 이렇게 하면, 도커를 활용하면서 빌드 폴더를 루트 소유로 만들지 않습니다.

```sh
# Create docker group (may not be required)
sudo groupadd docker
# Add your user to the docker group.
sudo usermod -aG docker $USER
# Log in/out again before using docker!
```

<a id="px4_containers"></a>

## Container Hierarchy

The available containers are listed below (from [Github](https://github.com/PX4/containers/blob/master/README.md#container-hierarchy)):

| 컨테이너                            | 설명                                   |
| ------------------------------- | ------------------------------------ |
| px4-dev-base                    | 모든 컨테이너에서 공통으로 활용하는 베이스 설치           |
| &emsp;px4-dev-nuttx             | NuttX 툴체인                            |
| &emsp;px4-dev-simulation        | NuttX 툴체인 + 모의시험 (jMAVSim, Gazebo)   |
| &emsp;&emsp;px4-dev-ros         | NuttX 툴체인, 모의시험 + ROS (incl. MAVROS) |
| &emsp;px4-dev-raspi             | 라즈베리 파이 툴체인                          |
| &emsp;px4-dev-snapdragon        | 퀄컴 스냅드래곤 비행 툴체인                      |
| &emsp;px4-dev-clang             | clang 도구                             |
| &emsp;&emsp;px4-dev-nuttx-clang | clang과 NuttX 도구                      |

The most recent version can be accessed using the `latest` tag: `px4io/px4-dev-nuttx:latest` (available tags are listed for each container on *hub.docker.com*. For example, the *px4-dev-ros* tags can be found [here](https://hub.docker.com/r/px4io/px4-dev-nuttx/tags)).

> **Tip** 보통 최근의 컨테이너를 활용해야 하나, 최신이 필요한 것은 아닙니다(변경이 너무 자주 일어나기 때문).

## 도커 컨테이너 활용

The following instructions show how to build PX4 source code on the host computer using a toolchain running in a docker container. The information assumes that you have already downloaded the PX4 source code to **src/PX4-Autopilot**, as shown:

```sh
mkdir src
cd src
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
```

### 보조 스크립트(docker_run.sh)

The easiest way to use the containers is via the [docker_run.sh](https://github.com/PX4/PX4-Autopilot/blob/master/Tools/docker_run.sh) helper script. This script takes a PX4 build command as an argument (e.g. `make tests`). It starts up docker with a recent version (hard coded) of the appropriate container and sensible environment settings.

For example, to build SITL you would call (from within the **/PX4-Autopilot** directory):

```sh
./Tools/docker_run.sh 'make px4_sitl_default'
```

Or to start a bash session using the NuttX toolchain:

    ./Tools/docker_run.sh 'bash'
    

> **Tip** *도커*에 대해 더 많이 알 필요도 없거니와 컨테이너가 뭘 활용하는지 생각할 필요가 없기 때문에 스크립트를 활용하시는 편이 쉽습니다. 그러나 일부분은 온전하지 않습니다! [아래 절](#manual_start)에서 다루는 내용을 통해 직접 접근하는 방식이 훨씬 유연하며, 스크립트에 어떤 문제가 있다면 오히려 아래와 같은 방식을 따라야합니다.

<a id="manual_start"></a>

### Calling Docker Manually

The syntax of a typical command is shown below. This runs a Docker container that has support for X forwarding (makes the simulation GUI available from inside the container). It maps the directory `<host_src>` from your computer to `<container_src>` inside the container and forwards the UDP port needed to connect *QGroundControl*. With the `-–privileged` option it will automatically have access to the devices on your host (e.g. a joystick and GPU). If you connect/disconnect a device you have to restart the container.

```sh
# 컨테이너의 xhost 접근 활성화
xhost +

# 도커 실행
docker run -it --privileged \
    --env=LOCAL_USER_ID="$(id -u)" \
    -v <host_src>:<container_src>:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e DISPLAY=:0 \
    -p 14570:14570/udp \
    --name=<local_container_name> <container>:<tag> <build_command>
```

Where,

* `<host_src>`: 컨테이너의 `<container_src>` 디렉터리에 대응할 호스트 컴퓨터의 디렉터리입니다. This should normally be the **PX4-Autopilot** directory.
* `<container_src>`: 컨테이너에 들어있는 공유 (소스) 디렉터리의 위치입니다.
* `<local_container_name>`: 만들어 둔 도커 컨테이너의 이름입니다. 컨테이너를 나중에 다시 참조해야 할 때 활용할 수 있습니다.
* `<container>:<tag>`: 시작할 컨테이너 이름과 버전입니다. 예시: `px4io/px4-dev-ros:2017-10-23`
* `<build_command>`: 새 컨테이너에서 실행할 명령입니다. 예시: `bash`는 컨테이너의 배시 셸을 여는데 사용하는 명령입니다.

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

### 컨테이너 재진입

The `docker run` command can only be used to create a new container. To get back into this container (which will retain your changes) simply do:

```sh
# start the container
docker start container_name
# open a new bash shell in this container
docker exec -it container_name bash
```

If you need multiple shells connected to the container, just open a new shell and execute that last command again.

### 컨테이너 정리

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

> **Note** 이중 중괄호 사이에 공백문자를 두어서는 안됩니다(gitbook의 인터페이스 렌더링 문제로 일부러 빈칸을 두었습니다).

### 문제 해결

#### 권한 오류

The container creates files as needed with a default user - typically "root". This can lead to permission errors where the user on the host computer is not able to access files created by the container.

The example above uses the line `--env=LOCAL_USER_ID="$(id -u)"` to create a user in the container with the same UID as the user on the host. This ensures that all files created within the container will be accessible on the host.

#### 그래픽 드라이버 문제

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

* OS X VMWare Fusion 환경에 Ubuntu 14.04 설치(GUI 지원 도커 컨테이너 병렬 실행시 X-Server 치명 오류 발생).

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