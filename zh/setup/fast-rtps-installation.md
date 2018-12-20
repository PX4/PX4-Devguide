# FastRTPS 安装

<img src="../../assets/fastrtps/eprosima_logo.png" style="float:left;" /> [eProsima Fast RTPS](http://eprosima-fast-rtps.readthedocs.io/en/latest/) 是 RTPS（实时发布订阅者）协议的 C++ 实现，它在不可靠的传输（如 UDP）上提供发布者-订阅者通信，由对象管理组（OMG）定义和维护社区。 RTPS 也是为数据分发服务（DDS）标准定义的有线互操作性协议，也是由 OMG 定义的。

PX4 使用 FastRTPS，使 RTPS 接口能够与板外组件（包括机器人和模拟器工具）共享 PX4 uORB 主题。 RTPS 是 DDS 的基本协议，是 OMG（对象管理组）提供实时发布/订阅中间件的标准，广泛应用于航空航天、国防和物联网应用。 它也被用作 ROS2 机器人工具包的中间件。 有关详细信息，请参阅：[RTPS/ROS2 接口：PX4-FastRTPS Bridge](../middleware/micrortps.md)。

<span></span>

> **Note** 本主题来源于官方的 [*eProsima Fast RTPS* 文件](http://eprosima-fast-rtps.readthedocs.io/en/latest/)。 有关详细信息，请参阅︰

* [要求](http://eprosima-fast-rtps.readthedocs.io/en/latest/requirements.html#requirements)
* [源码安装](http://eprosima-fast-rtps.readthedocs.io/en/latest/sources.html#installation-from-sources)
* [二进制安装](http://eprosima-fast-rtps.readthedocs.io/en/latest/binaries.html#installation-from-binaries)

## 标准安装

Fast RTPS is installed as part of the PX4 developer environment on some platforms:

* [Development Environment on Mac](../setup/dev_env_mac.md) (FastRTPS included in common tools)
* [Development Environment on Linux](../setup/dev_env_linux.md) (FastRTPS included in install scripts)
* [Development Environment on Windows > Bash on Windows](/setup/dev_env_windows.md#bash-on-windows-new) (FastRTPS included in install script)

The instruction below are useful for adding FastRTPS support in other environments.

## Requirements

*eProsima Fast RTPS* requires the following packages to work.

### Run Dependencies

#### Java

Java is required to use our built-in code generation tool - *fastrtpsgen*. [Java JDK 8](http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html) is recommended.

### Windows 7 32-bit and 64-bit

#### Visual C++ 2013 or 2015 Redistributable Package

*eProsima Fast RTPS* requires the Visual C++ Redistributable packages for the Visual Studio version you chose during the installation or compilation. The installer gives you the option of downloading and installing them.

## Installation from Sources

Clone the project from Github:

```sh
$ git clone https://github.com/eProsima/Fast-RTPS
$ mkdir Fast-RTPS/build && cd Fast-RTPS/build
```

> **Note** You may need to [install Gradle](https://gradle.org/install/) to build the source (e.g. this is true on vanilla Fedora Linux). A build warning will be displayed if this is the case.

If you are on Linux, execute:

```sh
$ cmake -DTHIRDPARTY=ON -DBUILD_JAVA=ON ..
$ make
$ sudo make install
```

This will install Fast RTPS to `/usr/local`. You can use `-DCMAKE_INSTALL_PREFIX=<path>` to install to a custom location. Afterwards make sure the `fastrtpsgen` application is in your `PATH`. You can check with `which
fastrtpsgen`.

If you are on Windows, choose your version of *Visual Studio*:

```sh
> cmake -G "Visual Studio 14 2015 Win64" -DTHIRDPARTY=ON -DBUILD_JAVA=ON ..
> cmake --build . --target install
```

If you want to compile the examples, you will need to add the argument `-DCOMPILE_EXAMPLES=ON` when calling *CMake*.

If you want to compile the performance tests, you will need to add the argument `-DPERFORMANCE_TESTS=ON` when calling *CMake*.

## Installation from Binaries

You can always download the latest binary release of *eProsima Fast RTPS* from the [company website](http://www.eprosima.com/).

Documentation on how to do this can be found here: [Installation from Binaries](http://eprosima-fast-rtps.readthedocs.io/en/latest/binaries.html#installation-from-binaries) (*eProsima Fast RTPS* official documentation)

### Windows 7 32-bit and 64-bit

Execute the installer and follow the instructions, choosing your preferred *Visual Studio* version and architecture when prompted.

#### Environmental Variables

*eProsima Fast RTPS* requires the following environmental variable setup in order to function properly

* `FASTRTPSHOME`: Root folder where *eProsima Fast RTPS* is installed.
* Additions to the `PATH`: the **/bin** folder and the subfolder for your Visual Studio version of choice should be appended to the PATH.

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