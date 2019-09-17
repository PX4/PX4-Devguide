# Fast RTPS Installation

<img src="../../assets/fastrtps/eprosima_logo.png" style="float:left;"/> [eProsima Fast RTPS](http://eprosima-fast-rtps.readthedocs.io/en/latest/) is a C++ implementation of the RTPS (Real Time Publish Subscribe) protocol, which provides publisher-subscriber communications over unreliable transports such as UDP, as defined and maintained by the Object Management Group (OMG) consortium.
RTPS is also the wire interoperability protocol defined for the Data Distribution Service (DDS) standard, again by the OMG.

Fast RTPS is used by PX4 to enable an RTPS interface allowing PX4 uORB topics to be shared with offboard components, including robotics and simulator tools.
RTPS is the underlying protocol of DDS, a standard from the OMG (Object Management Group) providing a real-time publish/subscribe middleware that is widely used in aerospace, defense and IoT applications. It has also been adopted as the middleware for the ROS2 robotics toolkit.
For more information see: [RTPS/ROS2 Interface: PX4-FastRTPS Bridge](../middleware/micrortps.md).

<span></span>
> **Note** This topic is derived from the official [*eProsima Fast RTPS* documentation](http://eprosima-fast-rtps.readthedocs.io/en/latest/). For more information see:
  - [Requirements](http://eprosima-fast-rtps.readthedocs.io/en/latest/requirements.html#requirements)
  - [Installation from Sources](http://eprosima-fast-rtps.readthedocs.io/en/latest/sources.html#installation-from-sources)
  - [Installation from Binaries](http://eprosima-fast-rtps.readthedocs.io/en/latest/binaries.html#installation-from-binaries)

## Standard Installations

Fast RTPS is installed as part of the PX4 developer environment on some platforms:

* [Development Environment on Mac](../setup/dev_env_mac.md) (FastRTPS included in common tools)
* [Development Environment on Linux](../setup/dev_env_linux.md) (FastRTPS included in install ROS install script but not NuttX/Simulator script)
* [Development Environment on Windows > Bash on Windows](../setup/dev_env_windows_bash_on_win.md) (FastRTPS included in install script)

The instruction below are useful for adding FastRTPS support in other environments.


## Requirements

*eProsima Fast RTPS* requires the following packages to work.


### Run Dependencies

#### Java

Java is required to use our built-in code generation tool - *fastrtpsgen*. [Java JDK 8](http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html) is recommended.


### Windows 7 32-bit and 64-bit

#### Visual C++ 2013 or 2015 Redistributable Package

*eProsima Fast RTPS* requires the Visual C++ Redistributable packages for the Visual Studio version you chose during the installation or compilation.
The installer gives you the option of downloading and installing them.



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
This will install Fast RTPS to `/usr/local`.
You can use `-DCMAKE_INSTALL_PREFIX=<path>` to install to a custom location.
Afterwards make sure the `fastrtpsgen` application is in your `PATH`.
You can check with `which fastrtpsgen`.

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

Extract the contents of the package.
It will contain both *eProsima Fast RTPS* and its required package *eProsima Fast CDR*. You will have follow the same procedure for both packages, starting with *Fast CDR*.

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
