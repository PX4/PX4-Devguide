#!/bin/bash

## Bash script for setting up a PX4 development environment on Arch Linux (as of July 5th, 2019)
## It can be used for installing simulators (only) or for installing the preconditions for Snapdragon Flight or Raspberry Pi.
##
## Installs:
## - Common dependencies libraries and tools as defined in `arch_sim_common_deps.sh`
## - Gazebo10 simulator

echo "Downloading dependent script 'arch_sim_common_deps.sh'"
# Source the arch_sim_common_deps.sh script directly from github
common_deps=$(wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/arch_sim_common_deps.sh -O -)
wget_return_code=$?
# If there was an error downloading the dependent script, we must warn the user and exit at this point.
if [[ $wget_return_code -ne 0 ]]; then echo "Error downloading 'arch_sim_common_deps.sh'. Sorry but I cannot proceed further :("; exit 1; fi
# Otherwise, source the downloaded script.
. <(echo "${common_deps}")


###
echo "Before proceeding, fix PGM in '/usr/lib/pkgconfig/openpgm-5.2.pc': 

   Replace 'Cflags: -I${includedir}/pgm-5.2 -I${libdir}/pgm-5.2/include'
   with 'Cflags: -I${includedir}/pgm-5.2'.
   "
read -p "When done, press Enter to continue."

# Gazebo (10) simulator
echo "Installing Gazebo 10"
yay -S urdfdom urdfdom-headers
yay -S gazebo

# Go to the firmware directory
cd $clone_dir/Firmware