# Arch와 CentOS에 Linux 설치 방법

!-- import docs ninja build system -->
{% include "_ninja_build_system.txt" %}


## 흔하지 않은 Linux 시스템에 설치하기

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

Note: python-pip와 screen 설치가 필요할 수 있습니다.

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

 multilib 저장소가 유효한지 확인합니다.

```sh
sudo pacman -S base-devel lib32-glibc git-core python-pyserial zip vim
```

[yaourt](https://wiki.archlinux.org/index.php/Yaourt#Installation)와 [Arch User Repository (AUR)](https://wiki.archlinux.org/index.php/Arch_User_Repository)를 위한 패키지 매니저 설치합니다.

이를 이용해서 다음과 같이 다운로드, 컴파일, 설치:

<div class="host-code"></div>

```sh
yaourt -S genromfs python-empy
```

#### Permissions

user는 "uucp" 그룹에 추가:

<div class="host-code"></div>

```sh
sudo usermod -a -G uucp $USER
```

이렇게 한 후에, 로그아웃을 하고 다시 로그인을 합니다.


> **Note** 변경한 내용이 적용되려면 로그아웃과 로그인이 필요! 장치를 뺐다가 다시 꽂는 것도!


### GCC 툴체인 설치
<!-- import GCC toolchain common documentation -->
{% include "_gcc_toolchain_installation.txt" %}
