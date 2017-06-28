# Fast RTPS installation ![](../../assets/fastrtps/eprosima_logo.png)

[_eprosima Fast RTPS_](http://eprosima-fast-rtps.readthedocs.io/en/latest/) is a C++ implementation of the RTPS (Real Time Publish Subscribe) protocol, which provides publisher-subscriber communications over unreliable transports such as UDP, as defined and maintained by the Object Management Group (OMG) consortium. RTPS is also the wire interoperability protocol defined for the Data Distribution Service (DDS) standard, again by the OMG.

**NOTE**: The information here was extracted from original [_eprosima Fast RTPS_ documentation](http://eprosima-fast-rtps.readthedocs.io/en/latest/)

## [Requirements](http://eprosima-fast-rtps.readthedocs.io/en/latest/requirements.html#requirements)

*eProsima Fast RTPS* requires the following packages to work.

### Run Dependencies

#### Java

Java is required to make use of our built-in code generation tool *fastrtpsgen*. It's recommended install [Java JDK 8](http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html).

### Windows 7 32-bit and 64-bit

#### Visual C++ 2013 or 2015 Redistributable Package

*eProsima Fast RTPS* requires the Visual C++ Redistributable packages for the Visual Studio version you choose during the installation or compilation. The installer gives you the option of downloading and installing them.

**NOTE**: For more information, please, visit [Requirements](http://eprosima-fast-rtps.readthedocs.io/en/latest/requirements.html#requirements)

## [Installation from Sources](http://eprosima-fast-rtps.readthedocs.io/en/latest/sources.html#installation-from-sources)


Clone the project from Github:

```sh
$ git clone https://github.com/eProsima/Fast-RTPS
$ mkdir Fast-RTPS/build && cd Fast-RTPS/build
```
If you are on Linux, execute:

```sh
$ cmake -DTHIRDPARTY=ON ..
$ make
$ sudo make install
```
If you are on Windows, choose your version of Visual Studio:

```sh
> cmake -G "Visual Studio 14 2015 Win64" -DTHIRDPARTY=ON ..
> cmake --build . --target install
```
If you want to compile the examples, you will need to add the argmuent `-DCOMPILE_EXAMPLES=ON` when calling CMake.

If you want to compile the performance tests, you will need to add the argument `-DPERFORMANCE_TESTS=ON` when calling CMake.

**NOTE**: For more information, please, visit [Installation from Sources](http://eprosima-fast-rtps.readthedocs.io/en/latest/sources.html#installation-from-sources)

## [Installation from Binaries](http://eprosima-fast-rtps.readthedocs.io/en/latest/binaries.html#installation-from-binaries)


You can always download the latest binary realese of *eProsima Fast RTPS* from the [company website](http://www.eprosima.com/).

### Windows 7 32-bit and 64-bit

Execute the installer and follow the instructions, choosing your preferred Visual Studio version and architecture when prompted.

#### Environmental Variables


*eProsima Fast RTPS* requires the following environmental variable setup in order to function properly

* FASTRTPSHOME: Root folder where *eProsima Fast RTPS* is installed.
* Additions to the PATH: the /bin folder and the subfolder for your Visual Studio version of choice should be appended to the PATH.

These variables are set automatically by checking the corresponding box during the installation process.

### Linux

Extract the contents of the package. It will containt both *eProsima Fast RTPS* and its required package *eProsima Fast CDR*. You will have follow the same procedure for both packages, starting with *Fast CDR*.

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
**NOTE**: For more information, please, visit [Installation from Binaries](http://eprosima-fast-rtps.readthedocs.io/en/latest/binaries.html#installation-from-binaries)
