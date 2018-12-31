# ArchLinux 上的开发环境

> **Note** These instructions allow you to build PX4 (without RTPS) for NuttX targets, using an unsupported version of GCCE from the package manager. The instructions have been tested on Antergos (an Arch Linux based distribution) as it is easier to set up than Arch Linux. 我们希望在不久的将来为本工具链提供经过全面测试的安装指南。（PS：译者实测时发现 ：Epel 源链接错误、部分依赖项无法使用 easy_install 的方式安装，只能使用 pip、系统自带 cmake 版本过低需要手动升级等问题，然后就弃坑了，欢迎 CentOS 大神折腾一下）

## 权限

将当前用户加入用户组 “uucp” ：

```sh
sudo usermod -a -G uucp $USER
```

然后注销并重新登录以使上述改动生效。

## 基于脚本的安装

> **Note** This script installs the (unsupported) latest GCCE from the package manager. MicroRTPS is not built.

Once ArchLinux is installed you can use the docker script [archlinux_install_script.sh](https://github.com/PX4/containers/blob/master/docker/px4-dev/scripts/archlinux_install_script.sh) to install all dependencies required for building PX4 firmware.

To install using this script, enter the following in a terminal:

```sh
wget https://raw.githubusercontent.com/PX4/containers/master/docker/px4-dev/scripts/archlinux_install_script.sh
sudo -s
source ./archlinux_install_script.sh
```

<!-- 
> Follow the instructions [below](#gcc-toolchain-installation) to install the supported version.
-->

## 手动安装

### 通用依赖

在终端输入以下命令进行依赖项的手动安装：

```sh
# 所有目标的通用依赖包
sudo pacman -Sy --noconfirm \
    base-devel make cmake ccache git \
    ninja python-pip tar unzip zip vim wget

# 安装 Python 依赖包
pip install serial empy numpy toml jinja2 pyyaml cerberus

# 安装 genromfs
wget https://sourceforge.net/projects/romfs/files/genromfs/0.5.2/genromfs-0.5.2.tar.gz
tar zxvf genromfs-0.5.2.tar.gz
cd genromfs-0.5.2 && make && make install && cd ..
rm genromfs-0.5.2.tar.gz genromfs-0.5.2 -r 
```

> **Note** *genromfs* is also available in the [Archlinux User Repository](https://aur.archlinux.org/packages/genromfs/) (AUR). To use this package, install [yaourt](https://archlinux.fr/yaourt-en) (Yet AnOther User Repository Tool) and then use it to download, compile and install *genromfs* as shown: 
> 
>     sh
>       yaourt -S genromfs

### GCCE 编译器

A GCC compiler is required to build for NuttX targets. Enter the command below to install the latest version from the package manager (unsupported).

    # 从包管理器安装编译器（不支持）
    sudo pacman -Sy --noconfirm \
        arm-none-eabi-gcc arm-none-eabi-newlib
    

*Alternatively*, the standard instructions for installing the **official** version are listed below.

> **Note** These are untested. Attempt them at your own risk!

<!-- import GCC toolchain common documentation -->

{% include "_gcc_toolchain_installation.md" %}