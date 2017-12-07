# Development Environment on ArchLinux

## Permissions

The user needs to be added to the group "uucp":

```sh
sudo usermod -a -G uucp $USER
```

After that, logging out and logging back in is needed.

> **Note** Log out and log in for changes to take effect! Also remove the device and plug it back in!

## Script-based Installation

Once ArchLinux is installed you can use our standard docker script 
[archlinux_install_script.sh](https://github.com/PX4/containers/blob/master/docker/px4-dev/scripts/archlinux_install_script.sh)
to install all dependencies required for building PX4 firmware.

To install using this script, enter the following in a terminal:
```sh
wget https://raw.githubusercontent.com/PX4/containers/master/docker/px4-dev/scripts/archlinux_install_script.sh
source ./archlinux_install_script.sh
```

## Common Dependencies

To install the dependencies manually, enter the following lines into a terminal.

```sh
# All dependencies for posix and nuttx targets
sudo pacman -Sy base-devel make cmake git-core \
    python-pip tar unzip zip vim wget \
    arm-none-eabi-gcc arm-none-eabi-newlib
    
# Install Python dependencies
pip install serial empy numpy toml jinja2

# Install genromfs
wget https://sourceforge.net/projects/romfs/files/genromfs/0.5.2/genromfs-0.5.2.tar.gz
tar zxvf genromfs-0.5.2.tar.gz
cd genromfs-0.5.2 && make && make install && cd ..
rm genromfs-0.5.2.tar.gz genromfs-0.5.2 -r 
```

<!-- import docs ninja build system -->
{% include "_ninja_build_system.txt" %}
