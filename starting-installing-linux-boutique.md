# Linux Installation Instructions for Arch and CentOS

## USB Device Configuration

Linux users need to explicitly allow access to the USB bus for JTAG programming adapters.

<aside class="note">
For Archlinux: replace the group plugdev with uucp in the following commands
</aside>

Run a simple ls in sudo mode to ensure the commands below succeed:

<div class="host-code"></div>

```sh
sudo ls
```

Then with sudo rights temporarily granted, run this command:

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

User needs to be added to the group plugdev:

<div class="host-code"></div>

```sh
sudo usermod -a -G plugdev $USER
```

## Installation Instructions for Uncommon Linux Systems

### CentOs

The build requires Python 2.7.5. Therefore as of this writing Centos 7 should be used. 
(For earlier Centos releases a side-by-side install of python v2.7.5 may be done. But it is not recommended because it can break yum.) 

The EPEL repositories are required for openocd libftdi-devel libftdi-python

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

Note:You may want to also install  python-pip and screen

#### Additional 32 bit libraries

Once the arm toolchain is installed test it with:

<div class="host-code"></div>

```sh
arm-none-eabi-gcc --version
```
If you receive the following message 

<div class="host-code"></div>

```sh
bash: gcc-arm-none-eabi-4_7-2014q2/bin/arm-none-eabi-gcc: /lib/ld-linux.so.2: bad ELF interpreter: No such file or directory
```
Then you will also need to install other 32-bit libraries glibc.i686 ncurses-libs.i686

<div class="host-code"></div>

```sh
sudo yum install glibc.i686 ncurses-libs.i686 
```
<aside class="note">
Pulling in ncurses-libs.i686 will pull in most of the other required 32 bit libraries. Centos 7 will install most all the PX4 related devices without the need for any added udev rules. The devices will be accessible to the predefined group ' dialout'. Therefore any references to adding udev rules can be ignored. The only requirement is that your user account is a member of the group 'dial out'
</aside>

### Arch Linux

<div class="host-code"></div>

```sh
sudo pacman -S base-devel lib32-glibc git-core python-pyserial zip python-empy
```

Install [yaourt](https://wiki.archlinux.org/index.php/Yaourt#Installation), the package manager for the [Arch User Repository (AUR)](https://wiki.archlinux.org/index.php/Arch_User_Repository).

Then use it to download, compile and install the following:

<div class="host-code"></div>

```sh
yaourt -S genromfs
```

#### Permissions

The user needs to be added to the group "uucp":

<div class="host-code"></div>

```sh
sudo usermod -a -G uucp $USER
```

After that, logging out and logging back in is needed.


<aside class="note">
Log out and log in for changes to take effect! Also remove the device and plug it back in!**
</aside>

### Toolchain Installation

Execute the script below to either install GCC 4.8.4 or 4.9.2:

<div class="host-code"></div>

```sh
pushd .
cd ~
wget https://launchpadlibrarian.net/186124160/gcc-arm-none-eabi-4_8-2014q3-20140805-linux.tar.bz2
tar -jxf gcc-arm-none-eabi-4_8-2014q3-20140805-linux.tar.bz2
exportline="export PATH=$HOME/gcc-arm-none-eabi-4_8-2014q3/bin:\$PATH"
if grep -Fxq "$exportline" ~/.profile; then echo nothing to do ; else echo $exportline >> ~/.profile; fi
. ~/.profile
popd
```

GCC 4.9:

<div class="host-code"></div>

```sh
pushd .
cd ~
wget https://launchpad.net/gcc-arm-embedded/4.9/4.9-2014-q4-major/+download/gcc-arm-none-eabi-4_9-2014q4-20141203-linux.tar.bz2
tar -jxf gcc-arm-none-eabi-4_9-2014q4-20141203-linux.tar.bz2
exportline="export PATH=$HOME/gcc-arm-none-eabi-4_9-2014q4/bin:\$PATH"
if grep -Fxq "$exportline" ~/.profile; then echo nothing to do ; else echo $exportline >> ~/.profile; fi
. ~/.profile
popd
```

<aside class="note">
If using Debian Linux, run this command:
</aside>

<div class="host-code"></div>

```sh
sudo dpkg --add-architecture i386
sudo apt-get update
```

Install the 32 bit support libraries (if running already on 32 bit this might fail and can be skipped):

<div class="host-code"></div>

```sh
sudo apt-get install libc6:i386 libgcc1:i386 gcc-4.6-base:i386 libstdc++5:i386 libstdc++6:i386
```

## Ninja Build System

Ninja is fast than Make and the PX4 CMake generators support it. Unfortunately Ubuntu carries only a very outdated version at this point. To install a recent version of [Ninja](https://github.com/martine/ninja), download the binary and add it to your path:

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

## Troubleshooting

### Version Test

Enter:

<div class="host-code"></div>

```sh
arm-none-eabi-gcc --version
```

The output should be something similar to:

<div class="host-code"></div>

```sh
arm-none-eabi-gcc (GNU Tools for ARM Embedded Processors) 4.7.4 20140401 (release) [ARM/embedded-4_7-branch revision 209195]
Copyright (C) 2012 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```

If you get:

<div class="host-code"></div>

```sh
arm-none-eabi-gcc --version
arm-none-eabi-gcc: No such file or directory
```
make sure you have the 32bit libs installed properly as described in the installation steps.
