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
```
To support simulation you you should also install
```sh
brew install px4-sim
```
```sh
# Quartz
brew cask install xquartz
```
```sh
# AdoptOpenJDK (Java8)
brew tap adoptopenjdk/openjdk
brew cask install adoptopenjdk8
brew install ant
export JAVA_HOME=$(/usr/libexec/java_home -v 1.8)
rm -rf Tools/jMAVSim/out
```

Make sure you have Python 3 installed.

```sh
brew install python3

# install required packages using pip3
pip3 install --user pyserial empy toml numpy pandas jinja2 pyyaml pyros-genmsg
```

## Additional Tools

After setting up the build/simulation toolchain, see [Additional Tools](../setup/generic_dev_tools.md) for information about other useful tools.

## Next Steps

Once you have finished setting up the environment, continue to the [build instructions](../setup/building_px4.md).

