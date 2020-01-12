# Development Environment on Mac

MacOS is a supported development platform for PX4. The following instructions set up an environment for building:
* NuttX-based hardware (Pixhawk, etc.)
* jMAVSim Smulation
* Gazebo 8 Simulation

> **Tip** To build other targets see: [Toolchain Installation > Supported Targets](../setup/dev_env.md#supported-targets).


## Preconditions

Increase the maximum allowed number of open files on macOS using the *Terminal* command:
```sh
ulimit -S -n 2048
```

> **Note**  At time of writing (December 2018) the master branch uses more than the default maximum allowed open files on macOS (256 in all running processes).
  As a *short term solution*, increasing the number of allowed open files to 300 should fix most problems.


## Homebrew Installation

The installation of Homebrew is quick and easy: [installation instructions](https://brew.sh).


## Common Tools

After installing Homebrew, run these commands in your shell to install the common tools:

```sh
brew tap PX4/px4
brew install px4-dev
# Optional, but recommended additional simulation tools:
brew install px4-sim
```

If the installation outputs an error message about missing requirements follow the instructions. Your system will be missing Java and Quartz:

```sh
brew cask install xquartz java
```

Install pip if you don't already have it and use it to install the required packages:

```sh
# get pip
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
python get-pip.py

# install required packages
sudo -H pip install pyserial empy toml numpy pandas jinja2 pyyaml
```

## Additional Tools

After setting up the build/simulation toolchain, see [Additional Tools](../setup/generic_dev_tools.md) for information about other useful tools.

## Next Steps

Once you have finished setting up the environment, continue to the [build instructions](../setup/building_px4.md).

