# Mac 개발 환경

MacOS는 PX4 지원 개발 환경입니다. 다음 절차는 빌드 환경을 설치합니다:

* NuttX-기반 하드웨어 (픽스호크 등)
* jMAVSim 시뮬레이션
* 가제보 시뮬레이션

> **Note** 다른 하드웨어를 대상으로 빌드하려면 [툴체인 설치 > 지원 대상](../setup/dev_env.md#supported-targets)을 살펴보십시오. 

<span></span>

> **Tip** 동영상 자습서는 여기에 있습니다: [Setting up your PX4 development environment on macOS](https://youtu.be/tMbMGiMs1cQ).

## 선행 조건

*터미널* 명령으로 macOS의 최대 개방 파일 수를 늘리십시오:

```sh
ulimit -S -n 2048
```

> **Note** 이 글을 작성하는 시점(2018년 12월) 마스터 브랜치에서는 macOS에서 허용하는 프로세스당 최대 개방 가능 파일 수(모든 실행 프로세스당 256개)를 넘게 활용합니다. *간단한 해결책*으로는, 허용 개방 파일 수를 늘리는 방법입니다.

## 홈브류(Homebrew) 설치

홈브루 설치 방법은 간단하고 쉽습니다: [설치 방법](https://brew.sh)

## 공통 도구

홈브류(Homebrew) 설치 후, 셸에서 이 명령을 실행하여 일반 도구를 설치하십시오:

```sh
brew tap PX4/px4
brew install px4-dev
```

파이썬 3를 설치했는지 확인하십시오.

```sh
brew install python3

# install required packages using pip3
pip3 install --user pyserial empy toml numpy pandas jinja2 pyyaml pyros-genmsg packaging
```

## 가제보 모의 시험 환경

가제보 SITL 모의시험 환경을 설치하려면:

```sh
brew cask install xquartz
brew install px4-sim-gazebo
```

## jMAVSim 모의 시험 환경

jMAVSim SITL 모의시험 환경을 활용하려면 자바 최신 버전(예: Java 14)을 우선 설치해야 합니다. [오라클에서 자바 14](https://www.oracle.com/java/technologies/javase-jdk14-downloads.html)를 다운로드 하거나 AdoptOpenJDK 탭을 활용할 수 있습니다:

```sh
brew tap AdoptOpenJDK/openjdk
brew cask install adoptopenjdk14
```

```sh
brew install px4-sim-jmavsim
```

> **Note** jMAVSim PX4 v1.11이전 버전은 Java 8이 필요합니다.

## 추가 도구

빌드 툴체인의 일부가 아닌 기타 유용한 개발 도구(IDE, GCS 등)의 내용을 살펴보려면 [추가 도구](../setup/generic_dev_tools.md) 를 살펴보십시오.

## 다음 단계

환경 구성이 끝나면, [빌드 설명서](../setup/building_px4.md)로 계속 진행하십시오.