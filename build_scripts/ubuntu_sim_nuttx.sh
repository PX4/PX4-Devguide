#!/bin/bash

## Bash script for setting up a PX4 development environment for Pixhawk/NuttX targets on Ubuntu LTS (16.04).
## It can be used for installing simulators and the NuttX toolchain.
##
## Installs:
## - Common dependencies libraries, tools, and Gazebo8 simulator as defined in `ubuntu_sim.sh`
## - NuttX toolchain (i.e. gcc compiler)

echo "Downloading dependent script 'ubuntu_sim.sh'"
# Source the ubuntu_sim.sh script directly from github
ubuntu_sim=$(wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim.sh -O -)
wget_return_code=$?
# If there was an error downloading the dependent script, we must warn the user and exit at this point.
if [[ $wget_return_code -ne 0 ]]; then echo "Error downloading 'ubuntu_sim.sh'. Sorry but I cannot proceed further :("; exit 1; fi
# Otherwise source the downloaded script.
. <(echo "${ubuntu_sim}")

# NuttX
sudo apt-get install python-serial openocd \
    flex bison libncurses5-dev autoconf texinfo \
    libftdi-dev libtool zlib1g-dev -y

# Clean up old GCC
sudo apt-get remove gcc-arm-none-eabi gdb-arm-none-eabi binutils-arm-none-eabi gcc-arm-embedded -y
sudo add-apt-repository --remove ppa:team-gcc-arm-embedded/ppa -y


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
