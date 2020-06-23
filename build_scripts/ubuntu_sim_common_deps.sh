#!/bin/bash

## Bash script for setting up a PX4 development environment on Ubuntu LTS (16.04).
## It can be used for installing simulators (only) or for installing the preconditions for Snapdragon Flight or Raspberry Pi.
##
## Installs:
## - Common dependencies and tools for all targets (including: Ninja build system, pyulog)
## - jMAVSim simulator dependencies
## - PX4/Firmware source (to ~/src/Firmware/)

# Preventing sudo timeout https://serverfault.com/a/833888
trap "exit" INT TERM; trap "kill 0" EXIT; sudo -v || exit $?; sleep 1; while true; do sleep 60; sudo -nv; done 2>/dev/null &

# Ubuntu Config
echo "Remove modemmanager"
sudo apt-get remove modemmanager -y
echo "Add user to dialout group for serial port access (reboot required)"
sudo usermod -a -G dialout $USER

# Common dependencies
echo "Installing common dependencies"
sudo apt-get update -y
sudo apt-get install git zip cmake build-essential genromfs ninja-build exiftool astyle -y
# make sure xxd is installed, dedicated xxd package since Ubuntu 18.04 but was squashed into vim-common before
which xxd || sudo apt install xxd -y || sudo apt-get install vim-common --no-install-recommends -y
# Required python packages
sudo apt-get install python-argparse python-empy python-toml python-numpy python-dev python-pip -y
sudo -H pip install --upgrade pip
sudo -H pip install pandas jinja2 pyserial pyyaml
# optional python tools
sudo -H pip install pyulog

# jMAVSim simulator dependencies
echo "Installing jMAVSim simulator dependencies"
sudo apt-get install ant openjdk-8-jdk openjdk-8-jre -y

