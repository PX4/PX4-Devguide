# Mac 上的开发环境

MacOS 是受支持的 PX4 开发平台。 根据本文的指示构建的开发环境可以用编译：

* 基于 NuttX 的硬件 (Pixhawk等)
* jMAVSim 仿真模拟
* Gazebo 8 仿真模拟

> **提示：** 若需要为其他平台进行编译请参考： [Toolchain Installation > Supported Targets](../setup/dev_env.md#supported-targets)。

## Preconditions

Increase the maximum allowed number of open files on macOS using the *Terminal* command:

```sh
ulimit -S -n 300
```

> **Note** At time of writing (December 2018) the master branch uses more than the default maximum allowed open files on macOS (256 in all running processes). As a *short term solution*, increasing the number of allowed open files to 300 should fix most problems.

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
sudo -H pip install pyserial empy toml numpy pandas jinja2 pyyaml
```

## Additional Tools

After setting up the build/simulation toolchain, see [Additional Tools](../setup/generic_dev_tools.md) for information about other useful tools.

## Next Steps

Once you have finished setting up the environment, continue to the [build instructions](../setup/building_px4.md).