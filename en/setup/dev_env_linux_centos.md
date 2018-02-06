# Development Environment on CentOS

> **Note** These instructions have not been tested with recent builds of PX4. We hope to provide fully tested instructions with the supported toolchain in the near future.

The build requires Python 2.7.5. Therefore as of this writing Centos 7 should be used.
(For earlier Centos releases a side-by-side install of python v2.7.5 may be done. But it is not recommended because it can break yum.)

## Common Dependencies

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

> **Note** You may want to also install  python-pip and screen

## GCC Toolchain Installation
<!-- import GCC toolchain common documentation -->
{% include "_gcc_toolchain_installation.txt" %}

## Additional 32 bit libraries

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


<!-- import docs ninja build system -->
{% include "_ninja_build_system.txt" %}
