# Mac 上的开发环境

MacOS 是受支持的 PX4 开发平台。 根据本文的指示构建的开发环境可以用编译：

* 基于 NuttX 的硬件 (Pixhawk等)
* jMAVSim 仿真模拟
* Gazebo Simulation

> **提示：** 若需要为其他平台进行编译请参考： [Toolchain Installation > Supported Targets](../setup/dev_env.md#supported-targets)。

## Preconditions

Increase the maximum allowed number of open files on macOS using the *Terminal* command:

```sh
ulimit -S -n 2048
```

> **Note** At time of writing (December 2018) the master branch uses more than the default maximum allowed open files on macOS (256 in all running processes). As a *short term solution*, increasing the number of allowed open files to 300 should fix most problems.

## Homebrew Installation

The installation of Homebrew is quick and easy: [installation instructions](https://brew.sh).

## Common Tools

After installing Homebrew, run these commands in your shell to install the common tools:

```sh
brew tap PX4/px4
brew install px4-dev
```

Make sure you have Python 3 installed.

```sh
brew install python3

# install required packages using pip3
pip3 install --user pyserial empy toml numpy pandas jinja2 pyyaml pyros-genmsg
```

## Gazebo Simulation

To install SITL simulation with Gazebo:

```sh
brew cask install xquartz
brew install px4-sim-gazebo
```

## jMAVSim Simulation

To install SITL simulation with jMAVSim:

```sh
brew tap AdoptOpenJDK/openjdk
brew cask install adoptopenjdk8
```

```sh
brew install px4-sim-jmavsim
```

## Additional Tools

See [Additional Tools](../setup/generic_dev_tools.md) for information about other useful development tools that are not part of the build toolchain (for example IDEs and GCSs).

## Next Steps

Once you have finished setting up the environment, continue to the [build instructions](../setup/building_px4.md).