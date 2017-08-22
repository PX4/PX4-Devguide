---
translated_page: https://github.com/PX4/Devguide/blob/master/en/setup/dev_env_mac.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# 安装文件和代码
第一步就是从Mac应用商店中安装Xcode。安装完成后打开一个新的终端并安装命令行工具：

```
xcode-select --install
```

## 安装Homebrew

推荐使用Mac OS X的 [Homebrew 包管理器](http://mxcl.github.com/homebrew/) 进行安装. Homebrew的安装十分便捷: [安装指南](http://mxcl.github.com/homebrew/).

安装好Homebrew以后,拷贝以下命令到终端命令行:



```sh
brew tap PX4/px4
brew tap PX4/simulation
brew update
brew install git bash-completion genromfs kconfig-frontends gcc-arm-none-eabi
brew install astyle cmake ninja
# simulation tools
brew install ant graphviz sdformat3 eigen protobuf
brew install opencv
```

然后安装我们需要的python包:



```sh
sudo easy_install pip
sudo pip install pyserial empy pandas jinja2
```

### Java for jMAVSim

如果你打算使用JMAVSim,需要同时安装[Java JDK 8](http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html).。

## 骁龙飞控

高通为Ubuntu提供了可靠的工具链。因此使用骁龙飞行平台的开发者应该安装一个Ubuntu虚拟机，并参考Linux下方法进行工具链的安装。PX4开发团队使用的虚拟机软件是VMWare，尤其是在VMWare对USB有了稳定的支持以后。

## 仿真

OS X 预装了CLANG. 因此无需再安装其他的编译器.

## 编辑器 / IDE

最后下载并安装 Qt Creator: [下载](http://www.qt.io/download-open-source/#section-6)

接下来进行 [第一次代码编译](../setup/building_px4.md)!
