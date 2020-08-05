# Fast RTPS 설치

<img alt="logo" src="../../assets/fastrtps/eprosima_logo.png" style="float:left;" /> [eProsima Fast RTPS](http://eprosima-fast-rtps.readthedocs.io/en/latest/)는 RTPS(Real Time Publish Subscribe) 규약을 C++로 구현한 것으로 개체 관리 그룹(Object Management Group, OMG) 컨소시엄에 의해 정의되고 관리되는 UDP와 같은 신뢰성이 좋지 않은 전송을 통해 게시자-가입자간의 통신을 제공합니다. 또한 RTPS는 역시나 OMG에 의한 데이터 분배 서비스(Data Distribution Service, DDS)로 정의된 유선 상호 운용성 규약입니다.

Fast RTPS는 PX4에서 RTPS 인터페이스가 활성화되도록 사용되어 PX4 uORB 주제를 허용함으로서 로봇 공학 및 시뮬레이터 도구를 포함한 보드 이외의 구성 요소와 공유되도록 합니다. RTPS는 DDS의 기반 규약으로 우주항공, 군사, 그리고 IoT 활용에 널리 사용되는 실시간 게시/가입 미들웨어를 제공하는 OMG(Object Management Group)의 표준입니다. 또한 ROS2 로봇 공학 툴킷을 위한 미들웨어로서 채택되었습니다. 더 많은 정보는 [RTPS/ROS2 인터페이스: PX4-FastRTPS 브릿지](../middleware/micrortps.md)를 보십시오.

> **Tip** 이 글을 작성하는 시점에 우분투 운영체제에서는 *소스 코드*로 FastRTPS 1.8.2 를 설치해야합니다.

<span></span>

> **Note** 이 주제의 내용은 [*eProsima Fast RTPS* 문서](http://eprosima-fast-rtps.readthedocs.io/en/latest/)의 공식 내용을 끌어왔습니다. 자세한 정보는 다음을 살펴보십시오:

* [요구사항](http://eprosima-fast-rtps.readthedocs.io/en/latest/requirements.html#requirements)
* [소스로 설치](http://eprosima-fast-rtps.readthedocs.io/en/latest/sources.html#installation-from-sources)
* [바이너리로 설치](http://eprosima-fast-rtps.readthedocs.io/en/latest/binaries.html#installation-from-binaries)

## 요구사항

*eProsima Fast RTPS*가 동작하려면 다음 꾸러미 설치가 필요합니다.

### 의존요소 실행

#### Java

내장 코드 생성 도구 *fastrtpsgen*을 활용하려면 Java 가 필요합니다. [Java JDK 8](http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html)을 추천합니다.

### Windows 7 32-bit와 64-bit

#### Visual C ++ 2013 또는 2015 재배포 가능 패키지

*eProsima Fast RTPS*는 설치 컴파일 단계에서 여러분이 선택한 Visual Studio 버전의 Visual C++ 재배포 가능 패키지가 필요합니다. 설치 관리자에서는 다운로드 및 설치 옵션을 제공합니다.

## 소스 코드로 설치하기

### Fast-RTPS

Github의 프로젝트를 복제하십시오:

```sh
$ git clone --recursive https://github.com/eProsima/Fast-RTPS.git -b 1.8.x ~/FastRTPS-1.8.2
$ cd ~/FastRTPS-1.8.2
$ mkdir build && cd build
```

> **Note** You may need to [install Gradle](https://gradle.org/install/) to build the source (e.g. this is true on vanilla Fedora Linux). A build warning will be displayed if this is the case.

리눅스에서 실행한다면, 다음 명령을 실행하십시오:

```sh
$ cmake -DTHIRDPARTY=ON -DSECURITY=ON ..
$ make
$ sudo make install
```

이 명령으로 FastRTPS를 `/usr/local`에 보안 통신 지원을 넣어 설치합니다. 개별 설정 위치에 설치하려면 `-DCMAKE_INSTALL_PREFIX=<path>` 인자를 사용하면 됩니다.

윈도우에서 설치한다면 *비주얼 스튜디오*의 버전을 선택하십시오.

```sh
> cmake -G "Visual Studio 14 2015 Win64" -DTHIRDPARTY=ON -DSECURITY=ON ..
> cmake --build . --target install
```

#### 컴파일 옵션

예제를 컴파일 하려면 *CMake*를 호출할 때 `-DCOMPILE_EXAMPLES=ON` 인자를 추가해야합니다.

컴파일 과정에서 성능 시험부를 빌드하려면 *CMake*를 호출할 때 `-DPERFORMANCE_TESTS=ON` 인자를 추가해야합니다.

### Fast-RTPS-Gen

*Fast-RTPS-Gen*은 Fast RTPS IDL 코드 생성 도구입니다. Fast RTPS 다음에 설치해야 하며 `PATH`에 `fastrtpsgen` 프로그램이 있는지 확인해야 합니다. `which fastrtpsgen` 명령으로 확인할 수 있습니다.

그리고 Fast-RTPS-Gen 버전 1.0.4를 설치하십시오(Gradle이 필요합니다):

    git clone --recursive https://github.com/eProsima/Fast-RTPS-Gen.git -b v1.0.4 ~/Fast-RTPS-Gen \
        && cd ~/Fast-RTPS-Gen \
        && gradle assemble \
        && gradle install
    

> **Note** "install" 단계에서 `sudo` 권한이 필요합니다.

## 바이너리 설치

> **Note** 바이너리를 쓸 수 있긴 하나, 소스 코드로부터 빌드하고 설치하는 방법을 추천드립니다. 바이너리로 받으면 필요 구성 요소와 의존 요소를 제자리에 위치하지 못할 수도 있습니다.

You can always download the latest binary release of *eProsima Fast RTPS* from the [company website](http://www.eprosima.com/).

Documentation on how to do this can be found here: [Installation from Binaries](http://eprosima-fast-rtps.readthedocs.io/en/latest/binaries.html#installation-from-binaries) (*eProsima Fast RTPS* official documentation)

### Windows 7 32-bit와 64-bit

Execute the installer and follow the instructions, choosing your preferred *Visual Studio* version and architecture when prompted.

#### 환경 변수

*eProsima Fast RTPS* requires the following environmental variable setup in order to function properly

* `FASTRTPSHOME`: Root folder where *eProsima Fast RTPS* is installed.
* `FASTRTPSGEN_DIR`: Root folder where *eProsima FastRTPSGen* is installed.
* Additions to the `PATH`: the **/bin** folder and the subfolder for your Visual Studio version of choice should be appended to the PATH.

These variables are set automatically by checking the corresponding box during the installation process.

### Linux

꾸러미 내용을 추출하십시오. *eProsima Fast RTPS*와 필요 꾸러미 *eProsima Fast CDR*이 다 들어있습니다. 두 꾸러미에 대해 동일한 과정을 진행합니다. *Fast CDR* 부터 시작하도록 하겠습니다.

컴파일 조건을 설정하십시오:

```sh
$ ./configure --libdir=/usr/lib
```

디버깅 심볼을 붙여 컴파일하려면(또한 자세한 메시지 출력 모드를 활성화함):

```sh
$ ./configure CXXFLAGS="-g -D__DEBUG"  --libdir=/usr/lib
```

프로젝트를 설정한 다음 라이브러리를 컴파일하고 설치하십시오.

```sh
$ sudo make install
```

#### 환경 변수

* `FASTRTPSGEN_DIR`: *eProsima FastRTPSGen*을 설치한 루트 폴더. 보통 기본 설치 디렉터리인 `/usr/local`로 설정합니다. 사용자가 `gradle install` 단계에서 다른 설치 디렉터리를 선택했다면, 이 값 역시 동일하게 설정해야합니다.