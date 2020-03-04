# Integration Testing using MAVSDK

PX4 can be tested end to end to using integration tests based on [MAVSDK](https://mavsdk.mavlink.io).

The tests are primarily developed against SITL for now and run in continuous integration (CI). However, they are meant to generalize to real tests eventually.

## Install the MAVSDK C++ Library

The tests need the MAVSDK C++ library installed system-wide (e.g. in `/usr/lib` or `/usr/local/lib`.

MAVSDK can either be installed as a prebuilt library or alternatively be built from source and installed system-wide.

### Installation of Prebuilt Library

#### Ubuntu

Download the matching .deb file (whichever is correct for your system) from [MAVSDK releases](https://github.com/mavlink/MAVSDK/releases).

Install it using `dpkg`, e.g.:

```sh
sudo dpkg -i mavsdk_0.23.0_ubuntu18.04_amd64.deb
```

#### Fedora

Download the matching .rpm file (whichever is correct for your system) from [MAVSDK releases](https://github.com/mavlink/MAVSDK/releases).

Install it using `rpm`, e.g.:

```sh
sudo rpm -U mavsdk-0.23.0-1.fc30-x86_64.rpm
```

#### Arch Linux

The library is available on [AUR](https://aur.archlinux.org/packages/mavsdk/) and can be installed e.g. using `yay`:

```sh
yay -S mavsdk
```

#### macOS

Install the library using [brew](https://brew.sh/):

```sh
brew install mavsdk
```

### Build and Install MAVSDK

Instead of installing the latest pre-built release, MAVSDK can also be built from sources. This enables you to build the library if there is is no prebuilt package for your platform, or because you need to use the latest [develop branch](https://github.com/mavlink/MAVSDK/tree/develop), or some custom branch or pull request. Also, you can specify the compile options, e.g. to select a debug build.

First fetch the sources from GitHub:

```sh
git clone https://github.com/mavlink/MAVSDK.git --recursive
```

Then build and install the library:
```
cd MAVSDK
cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_BACKEND=OFF -Bbuild -H. && cmake --build build -j 8 && sudo cmake --build build --target install
```

## Prepare PX4 Code

To build the PX4 code, use:

```sh
DONT_RUN=1 make px4_sitl gazebo mavsdk_tests
```

### Run All PX4 Tests

To run all tests, use:

```sh
test/mavsdk_tests/mavsdk_test_runner.py --speed-factor 20 --gui
```

To see all possible command line arguments, check out:

```sh
test/mavsdk_tests/mavsdk_test_runner.py -h
```
