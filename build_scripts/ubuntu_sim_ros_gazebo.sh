#!/bin/bash

## Bash script for setting up a ROS/Gazebo development environment for PX4 on Ubuntu LTS (16.04). 
## It installs the common dependencies for all targets (including Qt Creator) and the ROS Kinetic/Gazebo 7 (the default).
##
## Installs:
## - Common dependencies libraries and tools as defined in `ubuntu_sim_common_deps.sh`
## - ROS Kinetic (including Gazebo7)
## - MAVROS

echo "Downloading dependent script 'ubuntu_sim_common_deps.sh'"
# Source the ubuntu_sim_common_deps.sh script directly from github
common_deps=$(wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_common_deps.sh -O -)
wget_return_code=$?
# If there was an error downloading the dependent script, we must warn the user and exit at this point.
if [[ $wget_return_code -ne 0 ]]; then echo "Error downloading 'ubuntu_sim_common_deps.sh'. Sorry but I cannot proceed further :("; exit 1; fi
# Otherwise source the downloaded script.
. <(echo "${common_deps}")

# Setup gazebo: ROS Kinetic includes Gazebo7, ROS Melodic includes Gazebo9
## Gazebo simulator dependencies
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y

## ROS Gazebo: http://wiki.ros.org/kinetic/Installation/Ubuntu
## Setup keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
## For keyserver connection problems substitute hkp://pgp.mit.edu:80 or hkp://keyserver.ubuntu.com:80 above.
sudo apt-get update
## Get ROS/Gazebo
if [[ $(lsb_release -sc) == *"bionic"* ]]; then
  sudo apt-get install ros-melodic-desktop-full -y
  source /opt/ros/melodic/setup.bash
elif [[ $(lsb_release -sc) == *"xenial"* ]]; then
  sudo apt-get install ros-kinetic-desktop-full -y
  source /opt/ros/kinetic/setup.bash
else
  echo "OS version detected as $(lsb_release -sc), could not find valid ROS distro"
fi
## Initialize rosdep
sudo rosdep init
rosdep update
## Setup environment variables
rossource="source /opt/ros/$ROS_DISTRO/setup.bash"
if grep -Fxq "$rossource" ~/.bashrc; then echo ROS setup.bash already in .bashrc;
else echo "$rossource" >> ~/.bashrc; fi
eval $rossource
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
if ! rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y; then
    # (Use echo to trim leading/trailing whitespaces from the unsupported OS name
    unsupported_os=$(echo $(rosdep db 2>&1| grep Unsupported | awk -F: '{print $2}'))
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y --os ubuntu:$(lsb_release -sc)
fi

echo "Installing geographiclib dependencies"
sudo apt-get install libgeographic-dev -y
sudo apt-get install geographiclib-tools -y
sudo pip install future

## Build!
catkin config --extend /opt/ros/$ROS_DISTRO
catkin build
## Re-source environment to reflect new packages/build environment
catkin_ws_source="source ~/catkin_ws/devel/setup.bash"
if grep -Fxq "$catkin_ws_source" ~/.bashrc; then echo ROS catkin_ws setup.bash already in .bashrc; 
else echo "$catkin_ws_source" >> ~/.bashrc; fi
eval $catkin_ws_source

# Go to the firmware directory
clone_dir=~/src
cd $clone_dir/Firmware

## Edit bashrc to contain firmware related paths
firmware_source="source ~/src/Firmware/Tools/setup_gazebo.bash ~/src/Firmware ~/src/Firmware/build/px4_sitl_default > /dev/null"
if grep -Fxq "$firmware_source" ~/.bashrc; then echo firmware setup already in .bashrc; 
else echo "$firmware_source" >> ~/.bashrc; fi
eval $firmware_source

ros_package='export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/tanja/src/Firmware:/home/tanja/src/Firmware/Tools/sitl_gazebo'
if grep -Fxq "$ros_package" ~/.bashrc; then echo ros package path already in .bashrc; 
else echo "$ros_package" >> ~/.bashrc; fi

if [[ ! -z $unsupported_os ]]; then
    >&2 echo -e "\033[31mYour OS ($unsupported_os) is unsupported. Assumed an Ubuntu 16.04 or 18.04 installation,"
    >&2 echo -e "and continued with the installation, but if things are not working as"
    >&2 echo -e "expected you have been warned."
fi
