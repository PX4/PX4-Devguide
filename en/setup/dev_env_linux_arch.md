# Development Environment on ArchLinux

## Permissions

The user needs to be added to the group "uucp":

```sh
sudo usermod -a -G uucp $USER
```

After that, logging out and logging back in is needed.

> **Note** Log out and log in for changes to take effect! Also remove the device and plug it back in!

## Common Dependencies

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

## GCC Toolchain Installation
<!-- import GCC toolchain common documentation -->
{% include "_gcc_toolchain_installation.txt" %}

<!-- import docs ninja build system -->
{% include "_ninja_build_system.txt" %}
