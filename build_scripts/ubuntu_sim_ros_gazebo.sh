#!/bin/bash

## Bash script for setting up a ROS/Gazebo development environment for PX4 on Ubuntu LTS (16.04). 
##
## It installs the common dependencies for all targets (including Qt Creator) and the ROS Kinetic/Gazebo 7 (the default).
##
## Installs:
## - Common dependencies and tools for all targets (including: Ninja build system, Qt Creator, pyulog)
## - FastRTPS and FastCDR
## - ROS Kinetic (including Gazebo7)
## - MAVROS
## - PX4/Firmware source (to ~/src/Firmware/)

# Ubuntu Config
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

# Common dependencies
echo "Installing common dependencies"
sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
sudo apt-get update
sudo apt-get install python-argparse git-core wget zip python-empy qtcreator cmake build-essential genromfs -y
# Required python packages
sudo apt-get install python-dev -y
sudo apt-get install python-pip
sudo -H pip install pandas jinja2
pip install pyserial
# optional python tools
pip install pyulog

sudo apt-get install ant openjdk-8-jdk openjdk-8-jre -y


# Install FastRTPS 1.5.0 and FastCDR-1.0.7
fastrtps_dir=$HOME/eProsima_FastRTPS-1.5.0-Linux
echo "Installing FastRTPS to: $fastrtps_dir"
if [ -d "$fastrtps_dir" ]
then
    echo " FastRTPS already installed."
else
    pushd .
    cd ~
    wget http://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-rtps/eprosima-fast-rtps-1-5-0/eprosima_fastrtps-1-5-0-linux-tar-gz
    mv eprosima_fastrtps-1-5-0-linux-tar-gz eprosima_fastrtps-1-5-0-linux.tar.gz
    tar -xzf eprosima_fastrtps-1-5-0-linux.tar.gz eProsima_FastRTPS-1.5.0-Linux/
    tar -xzf eprosima_fastrtps-1-5-0-linux.tar.gz requiredcomponents
    tar -xzf requiredcomponents/eProsima_FastCDR-1.0.7-Linux.tar.gz
    cd eProsima_FastCDR-1.0.7-Linux; ./configure --libdir=/usr/lib; make; sudo make install
    cd ..
    cd eProsima_FastRTPS-1.5.0-Linux; ./configure --libdir=/usr/lib; make; sudo make install
    popd
fi


# ROS Kinetic/Gazebo (ROS Kinetic includes Gazebo7 by default)
## Gazebo dependencies
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y

## ROS Gazebo: http://wiki.ros.org/kinetic/Installation/Ubuntu
## Setup keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
## For keyserver connection problems substitute hkp://pgp.mit.edu:80 or hkp://keyserver.ubuntu.com:80 above.
sudo apt-get update
## Get ROS/Gazebo
sudo apt-get install ros-kinetic-desktop-full -y
## Initialize rosdep
sudo rosdep init
rosdep update
## Setup environment variables
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
## Get rosinstall
sudo apt-get install python-rosinstall -y

# MAVROS: https://dev.px4.io/en/ros/mavros_installation.html
## Create catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws

## Install dependencies
sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools -y

## Initialise wstool
wstool init ~/catkin_ws/src

## Build MAVROS
### Get source (upstream - released)
rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
### Get latest released mavlink package
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
### Setup workspace & install deps
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
## Build!
catkin build
## Re-source environment to reflect new packages/build environment
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc


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
    cd Firmware
fi
cd $clone_dir/Firmware
