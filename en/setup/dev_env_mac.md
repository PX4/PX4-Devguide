# Development Environment on Mac

Mac OS X is the main development platform for PX4. The following instructions set up a development environment for building:
* NuttX-based hardware (Pixhawk, etc.)
* jMAVSim Smulation
* Gazebo 8 Simulation

> **Tip** To build other targets see: [Toolchain Installation > Supported Targets](../setup/dev_env.md#supported-targets).


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
sudo -H pip install pyserial empy toml numpy pandas jinja2
```

<!-- import docs for other tools and next steps. -->
{% include "_addition_dev_tools.txt" %}

