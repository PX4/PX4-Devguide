#!/bin/bash

# Ubuntu Config
sudo apt-get remove modemmanager -y

# Ninja build system
mkdir -p $HOME/ninja
cd $HOME/ninja
wget https://github.com/martine/ninja/releases/download/v1.6.0/ninja-linux.zip
unzip ninja-linux.zip
rm ninja-linux.zip
exportline="export PATH=$HOME/ninja:\$PATH"
if grep -Fxq "$exportline" ~/.profile; then echo nothing to do ; else echo $exportline >> ~/.profile; fi
. ~/.profile

# Common Dependencies
sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
sudo apt-get update
sudo apt-get install python-argparse git-core wget zip \
    python-empy qtcreator cmake build-essential genromfs -y
# required python packages
sudo apt-get install python-pip
sudo -H pip install pandas jinja2

# jMAVSim simulator
sudo apt-get install ant openjdk-8-jdk openjdk-8-jre -y

# Gazebo simulator
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
## Setup keys
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
## Update the debian database:
sudo apt-get update -y
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

# Install GCC 5.4
pushd .
cd ~
wget https://launchpad.net/gcc-arm-embedded/5.0/5-2016-q2-update/+download/gcc-arm-none-eabi-5_4-2016q2-20160622-linux.tar.bz2
tar -jxf gcc-arm-none-eabi-5_4-2016q2-20160622-linux.tar.bz2
exportline="export PATH=$HOME/gcc-arm-none-eabi-5_4-2016q2/bin:\$PATH"
if grep -Fxq "$exportline" ~/.bash_profile; then echo nothing to do ; else echo $exportline >> ~/.profile; fi
popd

# Clone PX4/Firmware
mkdir -p ~/src
cd ~/src
git clone https://github.com/PX4/Firmware.git
cd Firmware
git submodule update --init --recursive