# Development Environment on Mac

The following instructions explain how to set up a development environment for NuttX-based hardware (Pixhawk, etc.) and Simulation (jMAVSim/Gazebo) targets. For other targets see: [Toolchain Installation > Supported Targets](../setup/dev_env.md#supported-targets).


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
sudo easy_install pip
sudo -H pip install pyserial empy pandas jinja2
```

<!-- import docs for other tools and next steps. -->
{% include "_addition_dev_tools.txt" %}

