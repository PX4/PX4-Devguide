#!/bin/bash

# Ubuntu Config
echo "Removing modemmanager"
sudo apt-get remove modemmanager -y

# Ninja build system
ninja_dir=$HOME/ninja
echo "Installing Ninja to: $ninja_dir."
if [ -d "$ninja_dir" ]
then
    echo " Ninja already installed."
else
    pushd .
    mkdir -p $ninja_dir
    cd $ninja_dir
    wget https://github.com/martine/ninja/releases/download/v1.6.0/ninja-linux.zip
    unzip ninja-linux.zip
    rm ninja-linux.zip
    exportline="export PATH=$ninja_dir:\$PATH"
    if grep -Fxq "$exportline" ~/.profile; then echo " Ninja already in path" ; else echo $exportline >> ~/.profile; fi
    . ~/.profile
    popd
fi


# Common Dependencies
echo "Installing common dependencies"
sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
sudo apt-get update
sudo apt-get install python-argparse git-core wget zip python-empy qtcreator cmake build-essential genromfs -y
# required python packages
sudo apt-get install python-dev -y
sudo apt-get install python-pip -y
sudo -H pip install pandas jinja2


# Gazebo simulator
echo "Installing Gazebo8"
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
## Setup keys
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
## Update the debian database:
sudo apt-get update
## Install Gazebo8
sudo apt-get install gazebo8 -y
## For developers (who work on top of Gazebo) one extra package
sudo apt-get install libgazebo8-dev


# NuttX
sudo apt-get install python-serial openocd \
    flex bison libncurses5-dev autoconf texinfo \
    libftdi-dev libtool zlib1g-dev -y

# Clean up old GCC
sudo apt-get remove gcc-arm-none-eabi gdb-arm-none-eabi binutils-arm-none-eabi gcc-arm-embedded
sudo add-apt-repository --remove ppa:team-gcc-arm-embedded/ppa

# Install GCC 5.4 - 64 Bit.
gcc_dir=$HOME/gcc-arm-none-eabi-5_4-2017q2
echo "Installing GCC to: $gcc_dir"
if [ -d "$gcc_dir" ]
then
    echo " GCC already installed."
else
    pushd .
    cd ~
    wget https://github.com/SolinGuo/arm-none-eabi-bash-on-win10-/raw/master/gcc-arm-none-eabi-5_4-2017q2-20170512-linux.tar.bz2
    tar -jxf gcc-arm-none-eabi-5_4-2017q2-20170512-linux.tar.bz2
    exportline="export PATH=$HOME/gcc-arm-none-eabi-5_4-2017q2/bin:\$PATH"
    if grep -Fxq "$exportline" ~/.profile; then echo " GCC path already set." ; else echo $exportline >> ~/.profile; fi
    . ~/.profile
    popd
    
    # Install 32 bit support libraries (ignore if fails)
    sudo dpkg --add-architecture i386
    sudo apt-get update
    sudo apt-get install libc6:i386 libgcc1:i386 libstdc++5:i386 libstdc++6:i386
    sudo apt-get install gcc-5.4-base:i386
fi


# jMAVSim simulator
echo "Installing jMAVSim"
sudo apt-get install ant -y
sudo add-apt-repository ppa:webupd8team/java
sudo apt-get update
sudo apt-get install oracle-java8-installer -y



# Clone PX4/Firmware
clone_dir=~/src
echo "Cloning PX4 to: $ninja_dir."
if [ -d "$clone_dir" ]
then
    echo " Firmware already cloned."
else
    mkdir -p $clone_dir
    cd ~/$clone_dir
    git clone https://github.com/PX4/Firmware.git
    cd Firmware
fi
cd $clone_dir/Firmware


#Reboot the computer (required before building)
#echo RESTART YOUR COMPUTER to complete installation of PX4 development toolchain
