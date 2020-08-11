## Ninja 构建系统

[Ninja](https://ninja-build.org/) 是一个比 *Make* 速度更快的构建系统， 且 PX4 *CMake* 生成器也支持该构建系统。 

在 Ubuntu Linux 上你从软件仓库中自动安装该构建系统。

```sh
sudo apt-get install ninja-build -y
```

其他 Linux 发行版系统的软件包管理器中可能并不包含 Ninja 软件包。
这种情况下你可以下载二进制文件然后将其加入操作系统的环境变量中：

```sh
mkdir -p $HOME/ninja
cd $HOME/ninja
wget https://github.com/martine/ninja/releases/download/v1.6.0/ninja-linux.zip
unzip ninja-linux.zip
rm ninja-linux.zip
exportline="export PATH=$HOME/ninja:\$PATH"
if grep -Fxq "$exportline" ~/.profile; then echo nothing to do ; else echo $exportline >> ~/.profile; fi
. ~/.profile
```
