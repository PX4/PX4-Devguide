# Mac 개발 환경

MacOS는 PX4 지원 개발 환경입니다. 다음 절차는 빌드 환경을 설치합니다:

* NuttX-기반 하드웨어 (픽스호크 등)
* jMAVSim 시뮬레이션
* 가제보 시뮬레이션

> **Note** 다른 하드웨어를 대상으로 빌드하려면 [툴체인 설치 > 지원 대상](../setup/dev_env.md#supported-targets)을 살펴보십시오. 

<span></span>

> **Tip** 동영상 자습서는 여기에 있습니다: [Setting up your PX4 development environment on macOS](https://youtu.be/tMbMGiMs1cQ).

## 홈브류(Homebrew) 설치

홈브루 설치 방법은 간단하고 쉽습니다: [설치 방법](https://brew.sh).

## 파일 열기 갯수 증가 ("LD: too many open files" 오류 처리)

PX4 툴체인에서는 ZSH 셸을 사용해야합니다. 셸을 사용중이라면, 다음 줄을 셸 프로파일에 추가하십시오:

이 파일을 만들거나 명령행을 뒤에 붙여넣습니다: `~/.zshenv` 그리고 다음 줄을 추가하십시오:

```sh
ulimit -S -n 2048
```

## 홈브류에서 파이썬 실행 대상 확인

아직 없다면, `~/.zshrc` 파일을 만들어 다음 줄을 추가하십시오:

```sh
# Point python to python 3 from Homebrew
alias python=/usr/local/bin/python3
# Point pip to python 3 pip
alias pip=/usr/local/bin/pip3
```

## 공통 도구

홈브류(Homebrew) 설치 후, 셸에서 이 명령을 실행하여 일반 도구를 설치하십시오:

```sh
brew tap PX4/px4
brew install px4-dev
```

필요한 파이썬 패키지를 설치하십시오

```sh
# install required packages using pip3
python3 -m pip install --user pyserial empy toml numpy pandas jinja2 pyyaml pyros-genmsg packaging
# if this fails with a permissions error, your Python install is in a system path - use this command instead:
sudo -H python3 -m pip install --user pyserial empy toml numpy pandas jinja2 pyyaml pyros-genmsg packaging
```

## 가제보 모의 시험 환경

가제보 SITL 모의시험 환경을 설치하려면:

```sh
brew cask install xquartz
brew install px4-sim-gazebo
```

## jMAVSim 모의 시험 환경

jMAVSim SITL 모의시험 환경을 활용하려면 자바 최신 버전(예: Java 14)을 우선 설치해야 합니다. You can either download [Java 14 from Oracle](https://www.oracle.com/java/technologies/javase-jdk14-downloads.html) or use the AdoptOpenJDK tap:

```sh
brew tap AdoptOpenJDK/openjdk
brew cask install adoptopenjdk14
```

```sh
brew install px4-sim-jmavsim
```

> **Note** jMAVSim for PX4 v1.11 and earlier required Java 8.

## Additional Tools

See [Additional Tools](../setup/generic_dev_tools.md) for information about other useful development tools that are not part of the build toolchain (for example IDEs and GCSs).

## Next Steps

Once you have finished setting up the environment, continue to the [build instructions](../setup/building_px4.md).