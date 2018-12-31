# Entorno de desarrollo en CentOS

> **Nota** Estas instrucciones no han sido probadas en recientes compilaciones de PX4. Esperamos proveer completas instrucciones probadas con el toolchain soportado en un futuro cercano.

La compilación requiere Python 2.7.5. Por lo tanto debería utilizar Centos 7. (Para versiones de CentOS anteriores una instalación "side-by-side" de python v2.7.5 podría hacerse. Pero no es recomendable porque podría romper yum.)

## Dependencias Comunes

Los repositorios EPEL son requeridos para openocd libftdi-devel libftdi-python

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

> **Nota** Podría querer instalar también python-pip y screen

## Instalación de Toolchain GCC

<!-- import GCC toolchain common documentation --> {% include "_gcc_toolchain_installation.md" %}

<!-- import docs ninja build system --> {% include "_ninja_build_system.md" %}