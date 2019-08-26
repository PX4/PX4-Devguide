# Entorno de desarrollo en ArchLinux

> **Nota** Estas instrucciones permiten compilar PX4 (sin RTPS) para objetivos NuttX, utilizando una versión no compatible de GCCE desde el gestor de paquetes. Las instrucciones han sido probadas en Antergos (una distribución Arch Linux base) ya que es más fácil de configurar que Arch Linux. Esperamos proveer completas instrucciones probadas con el toolchain soportado en un futuro cercano.

## Permisos

El usuario necesita ser agregado al grupo "uucp":

```sh
sudo usermod -a -G uucp $USER
```

Cierre sesión y vuelva a entrar para que los cambios tengan efecto.

## Instalación basada en Script

> **Nota** Este script instala el (no compatible) último GCCE desde el gestor de paquetes. MicroRTPS no es compilado.

On Arch Linux there is currently no script to install the dependencies. You can, however, check how the [Arch Linux Dockerfile](https://github.com/PX4/containers/blob/master/docker/px4-dev/Dockerfile_base-archlinux) for reference.

## Instalación manual

### Dependencias Comunes

To install the dependencies manually, enter the following lines into a terminal.

```sh
# Common dependencies for all targets
sudo pacman -Sy --noconfirm \
    base-devel make cmake ccache git ant \
    ninja python-pip tar unzip zip vim wget

# Install Python dependencies
pip install serial empy numpy toml jinja2 pyyaml cerberus

# Install genromfs
wget https://sourceforge.net/projects/romfs/files/genromfs/0.5.2/genromfs-0.5.2.tar.gz
tar zxvf genromfs-0.5.2.tar.gz
cd genromfs-0.5.2 && make && make install && cd ..
rm genromfs-0.5.2.tar.gz genromfs-0.5.2 -r 
```

> **Nota** *genromfs* esta también disponible en el [Repositorio de Usuario Archlinux](https://aur.archlinux.org/packages/genromfs/) (AUR). Para usar este paquete, instala [yaourt](https://archlinux.fr/yaourt-en) (Yet AnOther User Repository Tool) y entonces úsalo para descargar, compilar e instalar*genromfs* como aparece a continuación: 
> 
>     sh
>       yaourt -S genromfs

### Compilador GCCE

A GCC compiler is required to build for NuttX targets. Enter the command below to install the latest version from the package manager (unsupported).

    # Compiler from package manager (unsupported)
    sudo pacman -Sy --noconfirm \
        arm-none-eabi-gcc arm-none-eabi-newlib
    

*Alternatively*, the standard instructions for installing the **official** version are listed below.

> **Nota** No estan probadas. Usalos bajo tu propia responsabilidad!

<!-- import GCC toolchain common documentation -->

{% include "_gcc_toolchain_installation.md" %}