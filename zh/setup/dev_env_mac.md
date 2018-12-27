# Mac 上的开发环境

MacOS 是受支持的 PX4 开发平台。 根据本文的指示构建的开发环境可以用编译：

* 基于 NuttX 的硬件 (Pixhawk等)
* jMAVSim 仿真模拟
* Gazebo 8 仿真模拟

> **提示：** 若需要为其他平台进行编译请参考： [Toolchain Installation > Supported Targets](../setup/dev_env.md#supported-targets)。

## Homebrew 安装

Homebrew 的安装非常简单迅速：[installation instructions](https://brew.sh)。

## 常用工具

Homebrew 安装完毕后，在你的 shell 界面输入如下命令安装常用工具：

```sh
brew tap PX4/px4
brew install px4-dev
# 可选，但建议安装额外的仿真模拟用工具
brew install px4-sim
```

如上述安装过程输出了依赖项缺失的错误，请遵循下文的指示进行操作。 你的系统应该是缺失 Java 和 Quartz ：

```sh
brew cask install xquartz java
```

如果您还没有安装 pip ，请安装并使用它来安装所需的软件包：

```sh
sudo easy_install pip
sudo -H pip install pyserial empy toml numpy pandas jinja2 pyyaml
```

## 额外工具

完成编译/仿真开发环境设置后，你可以从 [Additional Tools](../setup/generic_dev_tools.md) 找到一些有用的“通用”开发工具。

## 后续步骤

设置完环境后，请转至 [build instructions](../setup/building_px4.md) 。