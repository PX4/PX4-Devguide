# 파일과 코드 설치하기

첫번째 단계는 Mac app 스토어에서 Xcode를 설치하는 것입니다. 일단 설치하고 나면, 새로운 터미널을 열어서 커맨드 라인 툴로 설치합니다:

<div class="host-code"></div>

```bash
xcode-select --install
```

## Homebrew 설치하기

Mac OS X를 사용하는 경우 [Homebrew 패키지 매니저](http://mxcl.github.com/homebrew/) 사용을 권장합니다. Homebrew 설치는 [설치 방법](http://mxcl.github.com/homebrew/)를 따라하면 쉽고 간편합니다.

Homebrew를 설치한 후에, 이 명령을 shell에 복사합니다. :

```sh
brew tap PX4/px4
brew tap PX4/simulation
brew update
brew install git bash-completion genromfs kconfig-frontends gcc-arm-none-eabi
brew install astyle cmake ninja
# simulation tools
brew install ant graphviz sdformat3 eigen protobuf
brew install opencv
```

다음으로 필요한 python 패키지를 설치합니다. :

```sh
sudo easy_install pip
sudo pip install pyserial empy pandas jinja2
```

### jMAVSim을 위한 Java

jMAVSim을 사용하고자 한다면, [Java JDK 8](http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html)을 설치해야 합니다.

## Snapdragon Flight

Snapdragon Flight로 개발하는 경우 Ubuntu VM을 사용해야 합니다. Qualcomm은 Ubuntu용으로만 툴을 제공하고 있습니다. PX4 개발팀은 VMWare으로 쭉 개발했으며 특히 USB가 안정적입니다.

## 시뮬레이션

OS X에는 CLANG이 미리 설치되어 있습니다. 추가 설치 단계가 필요하지 않습니다.

## Editor / IDE

이제 마지막으로 Qt Creator app을 다운로드 및 설치합니다. :
[Download](http://www.qt.io/download-open-source/#section-6)

이제 [처음 빌드하기](../setup/building_px4.md)를 이어서 실행하면 됩니다!
