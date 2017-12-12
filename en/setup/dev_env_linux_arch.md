# Development Environment on ArchLinux

## Permissions

The user needs to be added to the group "uucp":

```sh
sudo usermod -a -G uucp $USER
```

Then log out and log in for changes to take effect.


## Script-based Installation

Once ArchLinux is installed you can use our standard docker script 
[archlinux_install_script.sh](https://github.com/PX4/containers/blob/master/docker/px4-dev/scripts/archlinux_install_script.sh)
to install all dependencies required for building PX4 firmware.

To install using this script, enter the following in a terminal:
```sh
wget https://raw.githubusercontent.com/PX4/containers/master/docker/px4-dev/scripts/archlinux_install_script.sh
sudo -s
source ./archlinux_install_script.sh
```

> **Note** At time of writing this script installs the latest GCCE from the package manager. 
> This version works for building NuttX firmware, but is not "supported". 

<!-- 
> Follow the instructions [below](#gcc-toolchain-installation) to install the supported version.
-->


## Common Dependencies

To install the dependencies manually, enter the following lines into a terminal.

```sh
# Common dependencies for all targets
sudo pacman -Sy --noconfirm \
    base-devel make cmake ccache git \
    ninja python-pip tar unzip zip vim wget
    
# Install Python dependencies
pip install serial empy numpy toml jinja2

# Install genromfs
wget https://sourceforge.net/projects/romfs/files/genromfs/0.5.2/genromfs-0.5.2.tar.gz
tar zxvf genromfs-0.5.2.tar.gz
cd genromfs-0.5.2 && make && make install && cd ..
rm genromfs-0.5.2.tar.gz genromfs-0.5.2 -r 
```

> **Note** *genromfs* is also available in the 
> [Archlinux User Repository](https://aur.archlinux.org/packages/genromfs/) (AUR).
> To use this package, install [yaourt](https://archlinux.fr/yaourt-en) (Yet AnOther User Repository Tool) 
> and then use it to download, compile and install *genromfs* as shown:
> ```sh
  yaourt -S genromfs
  ```

## GCCE Compiler

A GCC compiler is required to build for NuttX targets. 
Enter the command below to install the latest version from the package manager.

```
# Compiler from package manager (unsupported)
sudo pacman -Sy --noconfirm \
    arm-none-eabi-gcc arm-none-eabi-newlib
```