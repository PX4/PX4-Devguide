# Fast RTPS 설치

<img alt="logo" src="../../assets/fastrtps/eprosima_logo.png" style="float:left;" /> [eProsima Fast RTPS](http://eprosima-fast-rtps.readthedocs.io/en/latest/)는 RTPS(Real Time Publish Subscribe) 프로토콜의 C++ 구현체입니다. 객체 관리 그룹(Object Management Group, OMG) 컨소시엄에서 기술을 정의하고 관리합니다. UDP와 같은 신뢰성이 낮은 전송 수단으로의 게시자-가입자간의 통신 기능을 제공합니다. 또한 RTPS는 OMG에 의한 데이터 배포 서비스(Data Distribution Service, DDS)로 정의한 유선 기반 상호 운용 프로토콜입니다.

PX4에서 활용하는 Fast RTPS는 RTPS 인터페이스에서 PX4 uORB 토픽을 로보틱스 도구와 모의시험 환경 도구 등의 오프보드 도구에 공유할 수 있게 합니다. RTPS는 DDS의 기반 프로토콜로 항공, 군사, IoT 활용에 널리 활용하도록 실시간 게시/가입 미들웨어를 제공하는 OMG(Object Management Group) 표준입니다. 또한 ROS2 로봇 공학 툴킷용 미들웨어로 채택했습니다. 더 많은 정보는 [RTPS/ROS2 인터페이스: PX4-FastRTPS 브릿지](../middleware/micrortps.md)를 보십시오.

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

> **Note** 소스코드를 빌드하려면 [Gradle을 설치](https://gradle.org/install/)해야 합니다. 이 경우 빌드 경고가 뜰 수 있습니다.

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
        && ./gradlew assemble \
        && sudo ./gradlew install
    

## 바이너리 설치

> **Note** Although the binaries are available, we recommend to build and install the code from source, given that the binaries may not come with required components and dependencies in place.

[업체 웹사이트](http://www.eprosima.com/)에서 *eProsima Fast RTPS*의 최신 바이너리 릴리즈를 얼마든지 다운로드할 수 있습니다.

이 과정을 진행하는 방법을 언급한 문서는 [Installation from Binaries](http://eprosima-fast-rtps.readthedocs.io/en/latest/binaries.html#installation-from-binaries) (*eProsima Fast RTPS* 공식 문서)에 있습니다.

### Windows 7 32-bit와 64-bit

설치 관리자를 실행하고 절차에 따라 적절한 *비주얼 스튜디오* 버전과 프로세서 아키텍처를 선택하십시오.

#### 환경 변수

*eProsima Fast RTPS*가 제대로 동작하려면 다음 환경 변수 설정이 필요합니다

* `FASTRTPSHOME`: *eProsima Fast RTPS*를 설치한 루트 폴더
* `FASTRTPSGEN_DIR`: *eProsima FastRTPSGen*을 설치한 루트 폴더
* `PATH`에 설정 추가: **/bin** 폴더와 선택한 비주얼 스튜디오 버전의 하위 폴더 역시 PATH에 등록해야합니다.

이 변수는 설치 과정에서 관련 상자를 표시하면 자동으로 설정합니다.

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