# Linux Installation Instructions for Arch and CentOS

<!-- import docs ninja build system -->
{% include "_ninja_build_system.txt" %}


## Installation Instructions for Uncommon Linux Systems

### CentOs

The build requires Python 2.7.5. Therefore as of this writing Centos 7 should be used.
(For earlier Centos releases a side-by-side install of python v2.7.5 may be done. But it is not recommended because it can break yum.)

The EPEL repositories are required for openocd libftdi-devel libftdi-python

```sh
wget https://dl.fedoraproject.org/pub/epel/7/x86_64/e/epel-release-7-5.noarch.rpm
sudo yum install epel-release-7-5.noarch.rpm
yum update
yum groupinstall “Development Tools”
yum install python-setuptools python-numpy
easy_install pyserial
easy_install pexpect
easy_install toml
yum install openocd libftdi-devel libftdi-python python-argparse flex bison-devel ncurses-devel ncurses-libs autoconf texinfo libtool zlib-devel cmake
```

Note:You may want to also install  python-pip and screen

#### Additional 32 bit libraries

Once the arm toolchain is installed test it with:

```sh
arm-none-eabi-gcc --version
```
If you receive the following message

```sh
bash: gcc-arm-none-eabi-4_7-2014q2/bin/arm-none-eabi-gcc: /lib/ld-linux.so.2: bad ELF interpreter: No such file or directory
```
Then you will also need to install other 32-bit libraries glibc.i686 ncurses-libs.i686

```sh
sudo yum install glibc.i686 ncurses-libs.i686
```

> **Note** Pulling in ncurses-libs.i686 will pull in most of the other required 32 bit libraries. Centos 7 will install most all the PX4 related devices without the need for any added udev rules. The devices will be accessible to the predefined group ' dialout'. Therefore any references to adding udev rules can be ignored. The only requirement is that your user account is a member of the group 'dial out'


### Arch Linux

Ensure you have the multilib repository enabled.

```sh
sudo pacman -S base-devel lib32-glibc git-core python-pyserial python-numpy python-pip zip vim
pip install --user toml
```

Install [yaourt](https://archlinux.fr/yaourt-en) (Yet AnOther User Repository Tool), a package manager for the [Arch User Repository (AUR)](https://wiki.archlinux.org/index.php/Arch_User_Repository).

Then use it to download, compile and install the following:

```sh
yaourt -S genromfs python-empy
```

#### Permissions

The user needs to be added to the group "uucp":

```sh
sudo usermod -a -G uucp $USER
```

After that, logging out and logging back in is needed.


> **Note** Log out and log in for changes to take effect! Also remove the device and plug it back in!**


### GCC Toolchain Installation
<!-- import GCC toolchain common documentation -->
{% include "_gcc_toolchain_installation.txt" %}
