#!/bin/bash

## Bash script for setting up a PX4 development environment for Pixhawk/NuttX targets on Arch Linux (as of July 5th, 2019).
## It can be used for installing simulators and the NuttX toolchain.
##
## Installs:
## - Common dependencies libraries, tools, and Gazebo8 simulator as defined in `arch_sim.sh`
## - NuttX toolchain (i.e. gcc compiler)

echo "Downloading dependent script 'arch_sim.sh'"
# Source the arch_sim.sh script directly from github
arch_sim=$(wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/arch_sim.sh -O -)
wget_return_code=$?
# If there was an error downloading the dependent script, we must warn the user and exit at this point.
if [[ $wget_return_code -ne 0 ]]; then echo "Error downloading 'arch_sim.sh'. Sorry but I cannot proceed further :("; exit 1; fi
# Otherwise source the downloaded script.
. <(echo "${arch_sim}")

# NuttX
sudo pacman -Sy python-pyserial openocd \
    flex bison libncurses5-dev autoconf texinfo \
    libftdi libtool zlib
yay -S ncurses5-compat-libs

# Clean up old GCC
yay -S gcc-arm-none-eabi-bin


# GNU Arm Embedded Toolchain: 7-2017-q4-major December 18, 2017
gcc_dir=$HOME/gcc-arm-none-eabi-7-2017-q4-major
echo "Installing GCC to: $gcc_dir"
if [ -d "$gcc_dir" ]
then
    echo " GCC already installed."
else
    pushd .
    cd ~    
    wget https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/7-2017q4/gcc-arm-none-eabi-7-2017-q4-major-linux.tar.bz2
    tar -jxf gcc-arm-none-eabi-7-2017-q4-major-linux.tar.bz2
    exportline="export PATH=$HOME/gcc-arm-none-eabi-7-2017-q4-major/bin:\$PATH"
    if grep -Fxq "$exportline" ~/.profile; then echo " GCC path already set." ; else echo $exportline >> ~/.profile; fi
    . ~/.profile
    popd
fi


# Go to the firmware directory
cd $clone_dir/Firmware

#Reboot the computer (required before building)
echo RESTART YOUR COMPUTER to complete installation of PX4 development toolchain