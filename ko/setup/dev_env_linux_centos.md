!REDIRECT "https://docs.px4.io/master/ko/dev_setup/dev_env_linux_centos.html"

# CentOS의 개발 환경

> **Note** 이 설명서 내용은 최근 PX4 빌드에 대해 시험해보지 않았습니다. 곧 지원 툴체인을 설명서를 따라 완전한 시험이 진행되길 빕니다.

빌드 과정에서 파이썬 2.7.5가 필요합니다. 때문에 이 문서를 쓰는 시점에서는 CentOS 7를 사용해야합니다. (이전 CentOS 릴리즈에서는 2.7.5 설치 정도면 족합니다. 그러나 yum이 동작하지 않을 수 있으므로 추천하지 않습니다.)

## 일반 의존요소

openocd, libftdi-devel libftdi-python 설치히 EPEL 저장소가 필요합니다

```sh
wget https://dl.fedoraproject.org/pub/epel/7/x86_64/e/epel-release-7-5.noarch.rpm
sudo yum install epel-release-7-5.noarch.rpm
yum update
yum groupinstall “Development Tools”
yum install python-setuptools python-numpy
easy_install pyserial
easy_install pexpect
easy_install toml
easy_install pyyaml
easy_install cerberus
yum install openocd libftdi-devel libftdi-python python-argparse flex bison-devel ncurses-devel ncurses-libs autoconf texinfo libtool zlib-devel cmake vim-common
```

> **Note** python-pip와 screen 설치도 필요합니다

## GCC 툴체인 설치

<!-- import GCC toolchain common documentation --> {% include "_gcc_toolchain_installation.md" %}

<!-- import docs ninja build system --> {% include "_ninja_build_system.md" %}