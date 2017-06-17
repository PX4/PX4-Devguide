# Fast RTPS 설치 ![](../../assets/fastrtps/eprosima_logo.png)

[_eprosima Fast RTPS_](http://eprosima-fast-rtps.readthedocs.io/en/latest/) RTPS (Real Time Publish Subscribe) 프로토콜을 C++로 구현한 것입니다. UDP처럼 신뢰성이 낮은 트랜스포트 상에서 publisher-subscriber 통신을 제공하는 경우로 OMG(Object Management Group) 컨소시엄에서 정의 및 관리하고 있습니다. RTPS는 DDS 표준을 위해 OMG가 정의한 유선 상호운영 프로토콜이기도 합니다.

**NOTE**: 여기 정보는 [_eprosima Fast RTPS_ documentation](http://eprosima-fast-rtps.readthedocs.io/en/latest/)에서 발췌한 것입니다.

## [요구사항](http://eprosima-fast-rtps.readthedocs.io/en/latest/requirements.html#requirements)

*eProsima Fast RTPS* 가 동작하려면 다음과 같은 패키지가 필요합니다.

### Run Dependencies

#### Java

[Java JDK 8](http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html) 설치를 추천합니다. Java는 빌트인 코드 생성 도구인 *fastrtpsgen*를 사용해야 합니다.

### Windows 7 32-bit와 64-bit

#### Visual C++ 2013 혹은 2015 Redistributable 패키지

*eProsima Fast RTPS*는 설치 및 컴파일하는 동안 Visual C++ Redistributable 패키지가 필요합니다. 인스톨러가 다운로드 및 설치에 관한 옵션을 제공합니다.

**NOTE**: 상세 정보는 [요구사항](http://eprosima-fast-rtps.readthedocs.io/en/latest/requirements.html#requirements)을 참고하세요.

## [소스에서 설치하기](http://eprosima-fast-rtps.readthedocs.io/en/latest/sources.html#installation-from-sources)

Github에서 프로젝트 clone :

```sh
$ git clone https://github.com/eProsima/Fast-RTPS
$ mkdir Fast-RTPS/build && cd Fast-RTPS/build
```
Linux인 경우 다음을 실행:

```sh
$ cmake -DTHIRDPARTY=ON ..
$ make
$ sudo make install
```

윈도우인 경우 Visual Studio 버전 선정:

```sh
> cmake -G "Visual Studio 14 2015 Win64" -DTHIRDPARTY=ON ..
> cmake --build . --target install
```
예제를 컴파일하려면, CMake를 호출할때 `-DCOMPILE_EXAMPLES=ON` 인자를 추가해야합니다.

성능 테스트를 원하면, CMake를 호출할때 `-DPERFORMANCE_TESTS=ON` 인자를 추가해야합니다.

**NOTE**: 상세 정보는 [소스에서 설치](http://eprosima-fast-rtps.readthedocs.io/en/latest/sources.html#installation-from-sources)를 참고하세요.

## [바이너리로 설치](http://eprosima-fast-rtps.readthedocs.io/en/latest/binaries.html#installation-from-binaries)

[사이트](http://www.eprosima.com/)에서 *eProsima Fast RTPS*의 최신 바이너리를 다운로드할 수 있습니다.

### Windows 7 32-bit와 64-bit

인스톨러를 실행하고 지시를 따릅니다. 프롬프트가 나오면 알맞는 Visual Studio 버전을 선택합니다.

#### 환경변수


*eProsima Fast RTPS*는 올바로 동작하려면 다음 환경변수 셋업을 따릅니다.

* FASTRTPSHOME: *eProsima Fast RTPS*가 설치되는 root 폴더.
* PATH에 추가: /bin 폴더와 선택한 Visual Studio 버전의 서브폴더를 PATH에 추가해야 합니다.

이 변수들은 설치하는 동안 관련 박스를 체크하면 자동으로 설정됩니다.

### Linux

패키지의 내용을 추출합니다. 여기에는 *eProsima Fast RTPS*와 *eProsima Fast CDR*가 포함되어 있습니다. 두개 패키지에 대해서 동일한 절차를 따르면 먼저 *Fast CDR*로 시작합니다.

컴파일 설정:

```sh
$ ./configure --libdir=/usr/lib
```
디버그 심볼로 컴파일을 원하는 경우 (verbose 모드 활성화):

```sh
$ ./configure CXXFLAGS="-g -D__DEBUG"  --libdir=/usr/lib
```
프로젝트 컴파일 설정 후에 라이브러리 설치:

```sh
$ sudo make install
```
**NOTE**: 추가 정보는 [바이너리로 설치](http://eprosima-fast-rtps.readthedocs.io/en/latest/binaries.html#installation-from-binaries) 참고하세요.
