# Arch와 CentOS에 Linux 설치 방법

## USB Device 설정

Linux 사용자는 JTAG 프로그래밍 아답터로 USB 버스에 접근할 수 있습니다.
Linux users need to explicitly allow access to the USB bus for JTAG programming adapters.

> **Note** Archlinux인 경우: 다음 명령으로 plugdev 그룹을 uucp로 대체합니다.

sudo 모드에서 간단한 ls명령을 실행해서 아래와 같이 확인합니다 :

<div class="host-code"></div>

```sh
sudo ls
```

임시로 sudo 권한을 얻은 다음에 다음 명령을 실행합니다. :

<div class="host-code"></div>

```sh
cat > $HOME/rule.tmp <<_EOF
# All 3D Robotics (includes PX4) devices
SUBSYSTEM=="usb", ATTR{idVendor}=="26AC", GROUP="plugdev"
# FTDI (and Black Magic Probe) Devices
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", GROUP="plugdev"
# Olimex Devices
SUBSYSTEM=="usb",  ATTR{idVendor}=="15ba", GROUP="plugdev"
_EOF
sudo mv $HOME/rule.tmp /etc/udev/rules.d/10-px4.rules
sudo /etc/init.d/udev restart
```

사용자를 plugdev 그룹에 추가해야 합니다. :

<div class="host-code"></div>

```sh
sudo usermod -a -G plugdev $USER
```

## 비일반적인 Linux 시스템에 설치하기

### CentOs

빌드에  Python 2.7.5이 필요합니다. 따라서 Centos 7를 사용해야 합니다.
(이전 Centos 배포판은 python v2.7.5와 함께 설치될 수도 있습니다. 하지만 yum이 맞지 않을 수 있기 때문에 추천하지 않습니다.)

EPEL 저장소는 openocd libftdi-devel libftdi-python가 필요합니다.

<div class="host-code"></div>

```sh
wget https://dl.fedoraproject.org/pub/epel/7/x86_64/e/epel-release-7-5.noarch.rpm
sudo yum install epel-release-7-5.noarch.rpm
yum update
yum groupinstall “Development Tools”
yum install python-setuptools
easy_install pyserial
easy_install pexpect
yum install openocd libftdi-devel libftdi-python python-argparse flex bison-devel ncurses-devel ncurses-libs autoconf texinfo libtool zlib-devel cmake
```

Note: python-pi와 screen 설치가 필요할 수 있습니다.

#### 추가할 32bit 라이브러리

arm 툴체인을 설치하면 이를 테스트 합니다. :

<div class="host-code"></div>

```sh
arm-none-eabi-gcc --version
```
다음과 같은 메시지가 나오면

<div class="host-code"></div>

```sh
bash: gcc-arm-none-eabi-4_7-2014q2/bin/arm-none-eabi-gcc: /lib/ld-linux.so.2: bad ELF interpreter: No such file or directory
```
다음으로 다른 32bit 라이브러리인 glibc.i686 ncurses-libs.i686를 설치해야 합니다.

<div class="host-code"></div>

```sh
sudo yum install glibc.i686 ncurses-libs.i686
```

> **Note** ncurses-libs.i686에서 필요한 것들은 다른 대부분의 32bit 라이브러리에서도 필요합니다. Centos 7은 추가 udev rules 없이도 PX4 관련된 거의 모든 장치를 설치합니다. 이 장치들은 미리 정의된 ' dialout' 그룹에 접근가능합니다. 따라서 udev rules 추가와 관련된 참조는 무시할 수 있습니다. 유일하게 필요한 것은 여러분의 사용자 계정이 'dial out' 그룹의 멤버로 되어 있어야 한다는 것입니다.

### Arch Linux

<div class="host-code"></div>

multilib 저장소가 활성화되어 있는지 확인합니다.

```sh
sudo pacman -S base-devel lib32-glibc git-core python-pyserial zip
```

[yaourt](https://wiki.archlinux.org/index.php/Yaourt#Installation)와 [Arch User Repository (AUR)](https://wiki.archlinux.org/index.php/Arch_User_Repository)용 패키지 매니저 설치.

다음으로 다운로드, 컴파일, 설치를 위해 다음을 사용합니다 :

<div class="host-code"></div>

```sh
yaourt -S genromfs python-empy
```

#### Permissions

사용자를 "uucp" 그룹에 추가합니다 :

<div class="host-code"></div>

```sh
sudo usermod -a -G uucp $USER
```

이제 재로그인이 필요합니다.

> **Note** 재로그인을 해야 동작합니다. 또한 해당 장치의 플러그를 재연결합니다.

### 툴체인 설치

아래 script를 실행하면 GCC 4.9 나 5.4 설치 :

**GCC 4.9**:

```sh
pushd .
cd ~
wget https://launchpad.net/gcc-arm-embedded/4.9/4.9-2015-q3-update/+download/gcc-arm-none-eabi-4_9-2015q3-20150921-linux.tar.bz2
tar -jxf gcc-arm-none-eabi-4_9-2015q3-20150921-linux.tar.bz2
exportline="export PATH=$HOME/gcc-arm-none-eabi-4_9-2015q3/bin:\$PATH"
if grep -Fxq "$exportline" ~/.bash_profile; then echo nothing to do ; else echo $exportline >> ~/.profile; fi
popd
```

**GCC 5.4**:

```sh
pushd .
cd ~
wget https://launchpad.net/gcc-arm-embedded/5.0/5-2016-q2-update/+download/gcc-arm-none-eabi-5_4-2016q2-20160622-linux.tar.bz2
tar -jxf gcc-arm-none-eabi-5_4-2016q2-20160622-linux.tar.bz2
exportline="export PATH=$HOME/gcc-arm-none-eabi-5_4-2016q2/bin:\$PATH"
if grep -Fxq "$exportline" ~/.bash_profile; then echo nothing to do ; else echo $exportline >> ~/.profile; fi
popd
```

이제 재시작합니다.

**Debian Linux 사용하는 경우 명령 수행:**

<div class="host-code"></div>

```sh
sudo dpkg --add-architecture i386
sudo apt-get update
```

**그리고 32 bit 지원 library 설치** (만약 32 bit OS를 실행중에 실패하는 경우 건너뛰기):

<div class="host-code"></div>

```sh
sudo apt-get install libc6:i386 libgcc1:i386 libstdc++5:i386 libstdc++6:i386
sudo apt-get install gcc-4.6-base:i386
```

## Ninja 빌드 시스템

Ninja는 Make보다 빠르고 PX4 CMake 생성자가 이를 지원합니다. 불행하게도 Ubuntu는 현재 시점에 예전 버전이 설치되어 있습니다. [Ninja](https://github.com/martine/ninja)의 최신 버전을 설치하려면 바이너리를 다운받고 이를 경로에 추가합니다 :

<div class="host-code"></div>

```sh
mkdir -p $HOME/ninja
cd $HOME/ninja
wget https://github.com/martine/ninja/releases/download/v1.6.0/ninja-linux.zip
unzip ninja-linux.zip
rm ninja-linux.zip
exportline="export PATH=$HOME/ninja:\$PATH"
if grep -Fxq "$exportline" ~/.profile; then echo nothing to do ; else echo $exportline >> ~/.profile; fi
. ~/.profile
```

## 문제해결

### 버전 테스트

입력:

<div class="host-code"></div>

```sh
arm-none-eabi-gcc --version
```

출력은 다음과 비슷한 형태 :

<div class="host-code"></div>

```sh
arm-none-eabi-gcc (GNU Tools for ARM Embedded Processors) 4.7.4 20140401 (release) [ARM/embedded-4_7-branch revision 209195]
Copyright (C) 2012 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```

다음과 같다면:

<div class="host-code"></div>

```sh
arm-none-eabi-gcc --version
arm-none-eabi-gcc: No such file or directory
```
설명한 것처럼 32bit 라이브러리가 제대로 설치되었는지 확인합니다.
