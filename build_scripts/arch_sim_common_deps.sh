#!/bin/bash

## Bash script for setting up a PX4 development environment on Arch Linux (as of July 5th, 2019)
## It can be used for installing simulators (only) or for installing the preconditions for Snapdragon Flight or Raspberry Pi.
##
## Installs:
## - Common dependencies and tools for all targets (including: Ninja build system, Qt Creator, pyulog)
## - FastRTPS and FastCDR
## - jMAVSim simulator dependencies
## - PX4/Firmware source (to ~/src/Firmware/)

# Preventing sudo timeout https://serverfault.com/a/833888
trap "exit" INT TERM; trap "kill 0" EXIT; sudo -v || exit $?; sleep 1; while true; do sleep 60; sudo -nv; done 2>/dev/null &

# Ask the user to install yay for AUR Package Manager
pacman -Qi yay > /dev/null
echo $yay_return_code
if [[ $yay_return_code -ne 0 ]]; 
then 
	echo "Installing yay..."; 
	git clone https://aur.archlinux.org/yay.git
	cd yay
	makepkg -si 
fi


# Remove modem manager
echo "We must first remove modemmanager"
sudo pacman -R modemmanager -y


# Common dependencies
# echo "Installing common dependencies"
sudo pacman -Syu
sudo pacman -S git zip qtcreator cmake base-devel astyle ninja wget
yay -S genromfs perl-image-exiftool 
# make sure xxd is installed
which xxd || sudo pacman -Sy xxd 
# Required python packages
sudo pacman -S python-argparse python-toml python-numpy python-pip
yay -S python-empy

sudo -H pip install --upgrade pip
sudo -H pip install pandas jinja2 pyserial pyyaml
# optional python tools
sudo -H pip install pyulog

# Install FastRTPS 1.7.1 and FastCDR-1.0.8
fastrtps_dir=$HOME/eProsima_FastRTPS-1.7.1-Linux
echo "Installing FastRTPS to: $fastrtps_dir"
if [ -d "$fastrtps_dir" ]
then
    echo " FastRTPS already installed."
else
    pushd .
    cd ~
    wget https://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-rtps/eprosima-fast-rtps-1-7-1/eprosima_fastrtps-1-7-1-linux-tar-gz -O eprosima_fastrtps-1-7-1-linux.tar.gz
    tar -xzf eprosima_fastrtps-1-7-1-linux.tar.gz eProsima_FastRTPS-1.7.1-Linux/
    tar -xzf eprosima_fastrtps-1-7-1-linux.tar.gz requiredcomponents
    tar -xzf requiredcomponents/eProsima_FastCDR-1.0.8-Linux.tar.gz
    cpucores=$(( $(lscpu | grep Core.*per.*socket | awk -F: '{print $2}') * $(lscpu | grep Socket\(s\) | awk -F: '{print $2}') ))
    (cd eProsima_FastCDR-1.0.8-Linux && ./configure --libdir=/usr/lib && make -j$cpucores) # && sudo make install)
    (cd eProsima_FastRTPS-1.7.1-Linux && ./configure --libdir=/usr/lib && make -j$cpucores) # && sudo make install)
    rm -rf requiredcomponents eprosima_fastrtps-1-7-1-linux.tar.gz
    popd
fi

# jMAVSim simulator dependencies
# echo "Installing jMAVSim simulator dependencies"
# sudo apt-get install ant openjdk-8-jdk openjdk-8-jre -y

# Clone PX4/Firmware
clone_dir=~/src
echo "Cloning PX4 to: $clone_dir."
if [ -d "$clone_dir" ]
then
   echo " Firmware already cloned."
else
    mkdir -p $clone_dir
    cd $clone_dir
    git clone https://github.com/PX4/Firmware.git
fi

# Setup catkin build config for python 3.7
echo "Go to your catkin workspace and run: 
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3.7 -DPYTHON_INCLUDE_DIR=/usr/include/python3.7 -DPYTHON_LIBRARY=/usr/lib/libpython3.7.so
"
