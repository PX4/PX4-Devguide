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

## 컨테이너 계층 {#px4_containers}

가용 컨테이너는 아래와 같습니다([Github](https://github.com/PX4/containers/blob/master/README.md#container-hierarchy)에 있음):

| 컨테이너                            | 설명                                   |
| ------------------------------- | ------------------------------------ |
| px4-dev-base                    | 모든 컨테이너에서 공통으로 활용하는 베이스 설치           |
| &emsp;px4-dev-nuttx             | NuttX 툴체인                            |
| &emsp;px4-dev-simulation        | NuttX 툴체인 + 모의시험 (jMAVSim, Gazebo)   |
| &emsp;&emsp;px4-dev-ros         | NuttX 툴체인, 모의시험 + ROS (incl. MAVROS) |
| &emsp;px4-dev-raspi             | 라즈베리 Pi 툴체인                          |
| &emsp;px4-dev-snapdragon        | 퀄컴 스냅드래곤 비행 툴체인                      |
| &emsp;px4-dev-clang             | clang 도구                             |
| &emsp;&emsp;px4-dev-nuttx-clang | clang과 NuttX 도구                      |

`px4io/px4-dev-nuttx:latest`와 같이 `latest` 태그를 활용하면 가장 최근의 버전에 접근할 수 있습니다. (가용 태그는 *hub.docker.com*의 각 컨테이너에 들어있습니다. 예를 들면, [여기](https://hub.docker.com/r/px4io/px4-dev-nuttx/tags)에서는, *px4-dev-ros*태그를 찾아볼 수 있습니다).

> **Tip** 보통 최근의 컨테이너를 활용해야 하나, 최신이 필요한 것은 아닙니다(변경이 너무 자주 일어나기 때문).

## 도커 컨테이너 활용

다음 절차는 도커 컨테이너에서 실행하는 툴체인으로 호스트 컴퓨터에서 PX4 소스 코드를 빌드하는 방법을 보여줍니다. PX4 소스 코드를 다음과 같이 **src/Firmware**에 이미 다운로드했음을 가정합니다:

```sh
mkdir src
cd src
git clone https://github.com/PX4/Firmware.git
cd Firmware
```

### 보조 스크립트(docker_run.sh)

컨테이너를 활용하는 가장 쉬운 방법은 [docker_run.sh](https://github.com/PX4/Firmware/blob/master/Tools/docker_run.sh) 보조 스크립트를 활용한 방법입니다. 이 스크립트는 PX4 빌드 명령을 인자 값으로 취합니다 (예: `make tests`). 명령을 통해 (하드 코딩한) 적절한 최근 버전의 컨테이너와 적당한 환경 설정 값으로 도커를 시작합니다.

예를 들어, SITL을 빌드하려면 다음 명령을 (**/Firmware** 디렉터리에서) 실행하십시오:

```sh
./Tools/docker_run.sh 'make px4_sitl_default'
```

또는 NuttX 툴체인으로 배시 세션을 시작하려면:

    ./Tools/docker_run.sh 'bash'
    

> **Tip** The script is easy because you don't need to know anything much about *Docker* or think about what container to use. However it is not particularly robust! The manual approach discussed in the [section below](#manual_start) is more flexible and should be used if you have any problems with the script.

### 도커 직접 호출 {#manual_start}

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

* `<host_src>`: The host computer directory to be mapped to `<container_src>` in the container. This should normally be the **Firmware** directory.
* `<container_src>`: The location of the shared (source) directory when inside the container.
* `<local_container_name>`: A name for the docker container being created. This can later be used if we need to reference the container again.
* `<container>:<tag>`: The container with version tag to start - e.g.: `px4io/px4-dev-ros:2017-10-23`.
* `<build_command>`: The command to invoke on the new container. E.g. `bash` is used to open a bash shell in the container.

아래의 보강 예제에서는 호스트 컴퓨터에서 배시 셸을 열고 **~/src/Firmware** 디렉터리를 공유하는 방법을 보여줍니다.

```sh
# 컨테이너의 xhost 접근 활성화
xhost +

# 도커 실행 후 배시 셸 열기
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

때로는 컨테이너를 함께 지워야 할 경우가 있습니다. 컨테이너 이름을 다음과 같이 붙이면 지울 수 있습니다:

```sh
docker rm mycontainer
```

이름을 기억할 수 없다면, 비활성 컨테이너 ID를 조회한 후 다음과 같이 삭제하십시오:

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

## 가상 머신 지원 {#virtual_machine}

Any recent Linux distribution should work.

The following configuration is tested:

* OS X with VMWare Fusion and Ubuntu 14.04 (Docker container with GUI support on Parallels make the X-Server crash).

**메모리**

Use at least 4GB memory for the virtual machine.

**컴파일 문제**

If compilation fails with errors like this:

```sh
The bug is not reproducible, so it is likely a hardware or OS problem.
c++: internal compiler error: Killed (program cc1plus)
```

Try disabling parallel builds.

**가상 머신 호스트에서 도커 제어 허용**

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