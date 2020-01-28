# FastRTPS 安装

<img src="../../assets/fastrtps/eprosima_logo.png" style="float:left;" /> [eProsima Fast RTPS](http://eprosima-fast-rtps.readthedocs.io/en/latest/) 是 RTPS（实时发布订阅者）协议的 C++ 实现，它在不可靠的传输（如 UDP）上提供发布者-订阅者通信，由对象管理组（OMG）定义和维护社区。 RTPS 也是为数据分发服务（DDS）标准定义的有线互操作性协议，也是由 OMG 定义的。

PX4 使用 FastRTPS，使 RTPS 接口能够与板外组件（包括机器人和模拟器工具）共享 PX4 uORB 主题。 RTPS 是 DDS 的基本协议，是 OMG（对象管理组）提供实时发布/订阅中间件的标准，广泛应用于航空航天、国防和物联网应用。 它也被用作 ROS2 机器人工具包的中间件。 有关详细信息，请参阅：[RTPS/ROS2 接口：PX4-FastRTPS Bridge](../middleware/micrortps.md)。

> **Tip** For Ubuntu, at time of writing, you will need to install Fast-RTPS 1.8.2 *from source*.

<span></span>

> **Note** This topic is derived from the official [*eProsima Fast RTPS* documentation](http://eprosima-fast-rtps.readthedocs.io/en/latest/). For more information see:

* [要求](http://eprosima-fast-rtps.readthedocs.io/en/latest/requirements.html#requirements)
* [源码安装](http://eprosima-fast-rtps.readthedocs.io/en/latest/sources.html#installation-from-sources)
* [二进制安装](http://eprosima-fast-rtps.readthedocs.io/en/latest/binaries.html#installation-from-binaries)

## 标准安装

在某些平台上，RTPS 作为 PX4 开发环境的一部分安装：

* [Mac 的开发环境（FastRTPS中包括常用工具）](../setup/dev_env_mac.md)
* [Development Environment on Windows > Bash on Windows](../setup/dev_env_windows_bash_on_win.md) (FastRTPS included in install script)

下面的说明对于在其他环境中添加 FastRTPS 支持非常有用。

## 要求

*eProsima Fast RTPS* 需要以下软件包才能正常工作。

### 依赖

#### Java

Java 需要使用我们内置的代码生成工具-*fastrtpsgen*。 建议使用 [Java JDK 8](http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html)。

### Windows 7 32位和64位

#### Visual C++ 2013 or 2015 Redistributable Package

*eProsima Fast RTPS</0 > 需要在安装或编译过程中选择的 Visual Studio 版本的 Visual C++ 可再发行包。 安装程序为您提供下载和安装它们的选项。</p> 

## 源码安装

在 Github 上下载项目：

```sh
$ git clone --recursive https://github.com/eProsima/Fast-RTPS.git -b 1.8.x ~/FastRTPS-1.8.2
$ cd ~/FastRTPS-1.8.2
$ mkdir build && cd build
```

> **Note** You may need to [install Gradle](https://gradle.org/install/) to build the source (e.g. this is true on vanilla Fedora Linux). A build warning will be displayed if this is the case.

如果您在 Linux 上，请执行：

```sh
$ cmake -DTHIRDPARTY=ON -DBUILD_JAVA=ON ..
$ make
$ sudo make install
```

这会将 FastRTPS 安装在 `/usr/local`。 You can use `-DCMAKE_INSTALL_PREFIX=<path>` to install to a custom location. Afterwards make sure the `fastrtpsgen` application is in your `PATH`. You can check with `which fastrtpsgen`.

Then install Fast-RTPS-Gen (Gradle is required for this):

    git clone --recursive https://github.com/eProsima/Fast-RTPS-Gen.git -b v1.0.2 ~/Fast-RTPS-Gen \
        && cd ~/Fast-RTPS-Gen \
        && gradle assemble \
        && sudo cp share/fastrtps/fastrtpsgen.jar /usr/local/share/fastrtps/ \
        && sudo cp scripts/fastrtpsgen /usr/local/bin/
    

If you are on Windows, choose your version of *Visual Studio*:

```sh
> cmake -G "Visual Studio 14 2015 Win64" -DTHIRDPARTY=ON -DBUILD_JAVA=ON ..
> cmake --build . --target install
```

If you want to compile the examples, you will need to add the argument `-DCOMPILE_EXAMPLES=ON` when calling *CMake*.

If you want to compile the performance tests, you will need to add the argument `-DPERFORMANCE_TESTS=ON` when calling *CMake*.

## 二进制安装

You can always download the latest binary release of *eProsima Fast RTPS* from the [company website](http://www.eprosima.com/).

Documentation on how to do this can be found here: [Installation from Binaries](http://eprosima-fast-rtps.readthedocs.io/en/latest/binaries.html#installation-from-binaries) (*eProsima Fast RTPS* official documentation)

### Windows 7 32位和64位

Execute the installer and follow the instructions, choosing your preferred *Visual Studio* version and architecture when prompted.

#### 环境变量

*eProsima Fast RTPS* requires the following environmental variable setup in order to function properly

* `FASTRTPSHOME`： *eProsima Fast RTPS* 根目录已安装。
* 添加到 `PATH`：所选 Visual Studio 版本的 **/bin** 文件夹和子文件夹应追加到 PATH 中。

These variables are set automatically by checking the corresponding box during the installation process.

### Linux

Extract the contents of the package. It will contain both *eProsima Fast RTPS* and its required package *eProsima Fast CDR*. You will have follow the same procedure for both packages, starting with *Fast CDR*.

Configure the compilation:

```sh
$ ./configure --libdir=/usr/lib
```

If you want to compile with debug symbols (which also enables verbose mode):

```sh
$ ./configure CXXFLAGS="-g -D__DEBUG"  --libdir=/usr/lib
```

After configuring the project compile and install the library:

```sh
$ sudo make install
```