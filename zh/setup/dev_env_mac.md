# Mac 上的开发环境

MacOS 是受支持的 PX4 开发平台。 根据本文的指示构建的开发环境可以用构建：

* 基于 NuttX 的硬件 (Pixhawk等)
* jMAVSim 仿真模拟
* Gazebo 8 仿真模拟

> **提示：** 若需要为其他平台进行编译请参考： [Toolchain Installation > Supported Targets](../setup/dev_env.md#supported-targets)。

## Homebrew 安装

Homebrew 的安装非常简单迅速：[installation instructions](https://brew.sh)。

## Common Tools

Homebrew 安装完毕后，在你的 shell 界面输入如下命令安装 common tools：

```sh
brew tap PX4/px4
brew install px4-dev
# 可选，但建议安装额外的仿真模拟用工具
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