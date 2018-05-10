# Windows Cygwin Toolchain

## Ready to use MSI Installer Download

Latest Download:

* [PX4 Windows Cygwin Toolchain 0.2 Download](https://s3-us-west-2.amazonaws.com/px4-tools/PX4+Windows+Cygwin+Toolchain/PX4+Windows+Cygwin+Toolchain+0.2.msi) (09.05.2018)

Legacy Versions (**deprecated**):

* [PX4 Windows Cygwin Toolchain 0.1 Download](https://s3-us-west-2.amazonaws.com/px4-tools/PX4+Windows+Cygwin+Toolchain/PX4+Windows+Cygwin+Toolchain+0.1.msi) (23.02.2018)

## Support / Known Problems

Tested and working:

* Building and running SITL with jMAVsim, generates a native windows binary px4.exe and has a lot better prformance than a VM
* Building and uploading NuttX builds like px4fmu-v2 and px4fmu-v4
* Style check with astyle with the command make format
* Command line auto completion
* The installer does **NOT** screw up your system and global path, it only modifies the selected installation directory e.g. `C:\PX4\` and uses a temporary local path 

[Known problems / Report your issue](https://github.com/orgs/PX4/projects/6)

## Installation Instructions

1. Download the latest version of the ready to use MSI installer above
1. Run it, choose your desired installation location (default C:\PX4\ strongly recommended), let it install
1. Make sure to tick the box in the end of the installation to clone the PX4 repository, build and run simulation with jMAVsim this simplifies the process to get you started. 

> **Tip** Don't worry if you missed the opportunity or the script failed (e.g. because you didn't have a working internet connection). You can also do that step manually on the terminal afterwards.

## Usage Instructions



```
# clones PX4 repository into the home folder & loads submodules in parallel
git clone --recursive -j8 https://github.com/PX4/Firmware.git
# switches into the Firmware repository folder
cd Firmware
# builds and runs SITL simulation with jMAVsim
make posix jmavsim
```

## Step by Step Manual Setup (for Toolchain Developers)

This section describes step by step how to reproduce all the core setup of the Cygwin toolchain which is in the ready to use installer.

1. Create the folders `C:\PX4\`, `C:\PX4\toolchain\` and `C:\PX4\home\`
1. Download the Cygwin installer file [setup-x86_64.exe](https://cygwin.com/setup-x86_64.exe) from the [official Cygwin website](https://cygwin.com/install.html)
1. Run the downloaded file and 


<!-- import docs for other tools and next steps. -->
{% include "_addition_dev_tools.txt" %}
