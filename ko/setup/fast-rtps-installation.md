# Fast RTPS 설치

<img src="../../assets/fastrtps/eprosima_logo.png" style="float:left;" /> [eProsima Fast RTPS](http://eprosima-fast-rtps.readthedocs.io/en/latest/)는 RTPS(Real Time Publish Subscribe) 규약을 C++로 구현한 것으로 개체 관리 그룹(Object Management Group, OMG) 컨소시엄에 의해 정의되고 관리되는 UDP와 같은 신뢰성이 좋지 않은 전송을 통해 게시자-가입자간의 통신을 제공합니다. 또한 RTPS는 역시나 OMG에 의한 데이터 분배 서비스(Data Distribution Service, DDS)로 정의된 유선 상호 운용성 규약입니다.

Fast RTPS는 PX4에서 RTPS 인터페이스가 활성화되도록 사용되어 PX4 uORB 주제를 허용함으로서 로봇 공학 및 시뮬레이터 도구를 포함한 보드 이외의 구성 요소와 공유되도록 합니다. RTPS는 DDS의 기반 규약으로 우주항공, 군사, 그리고 IoT 활용에 널리 사용되는 실시간 게시/가입 미들웨어를 제공하는 OMG(Object Management Group)의 표준입니다. 또한 ROS2 로봇 공학 툴킷을 위한 미들웨어로서 채택되었습니다. 더 많은 정보는 [RTPS/ROS2 인터페이스: PX4-FastRTPS 브릿지](../middleware/micrortps.md)를 보십시오.

> **Tip** For Ubuntu, at time of writing, you will need to install Fast-RTPS 1.8.2 *from source*.

<span></span>

> **Note** This topic is derived from the official [*eProsima Fast RTPS* documentation](http://eprosima-fast-rtps.readthedocs.io/en/latest/). For more information see:

* [요구사항](http://eprosima-fast-rtps.readthedocs.io/en/latest/requirements.html#requirements)
* [소스로 설치](http://eprosima-fast-rtps.readthedocs.io/en/latest/sources.html#installation-from-sources)
* [바이너리로 설치](http://eprosima-fast-rtps.readthedocs.io/en/latest/binaries.html#installation-from-binaries)

## 표준 설치

Fast RTPS는 아래와 같은 일부 플랫폼에서 PX4 개발자 환경의 일부로 설치됩니다.

* [Mac에서의 개발 환경](../setup/dev_env_mac.md) (Fast RTPS는 공통 도구에 포함되어 있음.)
* [Development Environment on Windows > Bash on Windows](../setup/dev_env_windows_bash_on_win.md) (FastRTPS included in install script)

아래의 설치 방법은 다른 환경에서 Fast RTPS 지원을 추가할 때 유용합니다.

## 요구사항

*eProsima Fast RTPS*는 작동을 위해 아래의 패키지들을 필요로 합니다.

### 의존성 실행

#### Java

Java는 내장된 코드 생성 도구인 *fastrtpsgen*을 사용해야합니다. [Java JDK 8](http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html)이 권장됩니다.

### Windows 7 32-bit와 64-bit

#### Visual C ++ 2013 또는 2015 재배포 가능 패키지

*eProsima Fast RTPS*는 설치 또는 컴파일 중에 선택한 Visual Studio 버전에 대한 Visual C ++ 재배포 가능 패키지가 필요합니다. 이 설치 프로그램은 다운로드 및 설치 옵션을 제공합니다.

## 소스로 설치

Github에서 프로젝트를 복사십시오.

```sh
$ git clone --recursive https://github.com/eProsima/Fast-RTPS.git -b 1.8.x ~/FastRTPS-1.8.2
$ cd ~/FastRTPS-1.8.2
$ mkdir build && cd build
```

> **Note** You may need to [install Gradle](https://gradle.org/install/) to build the source (e.g. this is true on vanilla Fedora Linux). A build warning will be displayed if this is the case.

Linux를 사용하는 경우 다음을 실행하십시오.

```sh
$ cmake -DTHIRDPARTY=ON -DBUILD_JAVA=ON ..
$ make
$ sudo make install
```

이렇게하면 Fast RTPS가 `/usr/local`에 설치됩니다. You can use `-DCMAKE_INSTALL_PREFIX=<path>` to install to a custom location. Afterwards make sure the `fastrtpsgen` application is in your `PATH`. You can check with `which fastrtpsgen`.

Then install Fast-RTPS-Gen (Gradle is required for this):

    git clone --recursive https://github.com/eProsima/Fast-RTPS-Gen.git -b v1.0.2 ~/Fast-RTPS-Gen \
        && cd ~/Fast-RTPS-Gen \
        && gradle assemble \
        && sudo cp share/fastrtps/fastrtpsgen.jar /usr/local/share/fastrtps/ \
        && sudo cp scripts/fastrtpsgen /usr/local/bin/
    

If you are on Windows, choose your version of *Visual Studio*:

```sh
> cmake -G "Visual Studio 14 2015 Win64" -DTHIRDPARTY=ON -DBUILD_JAVA=ON ..
> cmake --build . --target install
```

If you want to compile the examples, you will need to add the argument `-DCOMPILE_EXAMPLES=ON` when calling *CMake*.

If you want to compile the performance tests, you will need to add the argument `-DPERFORMANCE_TESTS=ON` when calling *CMake*.

## 바이너리로 설치

You can always download the latest binary release of *eProsima Fast RTPS* from the [company website](http://www.eprosima.com/).

Documentation on how to do this can be found here: [Installation from Binaries](http://eprosima-fast-rtps.readthedocs.io/en/latest/binaries.html#installation-from-binaries) (*eProsima Fast RTPS* official documentation)

### Windows 7 32-bit 와 64-bit

Execute the installer and follow the instructions, choosing your preferred *Visual Studio* version and architecture when prompted.

#### 환경 변수

*eProsima Fast RTPS* requires the following environmental variable setup in order to function properly

* `FASTRTPSHOME`: *eProsima Fast RTPS*가 설치된 루트 폴더.
* `PATH`에 추가: Visual Studio 버전의 **/bin** 폴더와 하위 폴더를 PATH에 추가해야합니다.

These variables are set automatically by checking the corresponding box during the installation process.

### Linux

Extract the contents of the package. It will contain both *eProsima Fast RTPS* and its required package *eProsima Fast CDR*. You will have follow the same procedure for both packages, starting with *Fast CDR*.

Configure the compilation:

```sh
$ ./configure --libdir=/usr/lib
```

If you want to compile with debug symbols (which also enables verbose mode):

```sh
$ ./configure CXXFLAGS="-g -D__DEBUG"  --libdir=/usr/lib
```

After configuring the project compile and install the library:

```sh
$ sudo make install
```