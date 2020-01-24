# Integration Testing using MAVSDK

PX4 can be tested end to end to using integration tests based on [MAVSDK](https://mavsdk.mavlink.io).

The tests are primarily developed against SITL for now and run in continuous integration (CI). However, they are meant to generalize to real tests eventually.

## Install the MAVSDK C++ library

The tests need the MAVSDK C++ library installed system-wide (e.g. in `/usr/lib` or `/usr/local/lib`.

### Installation of prebuilt library

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

#### macOS

Install the library using [brew](https://brew.sh/):

```sh
brew install mavsdk
```

## Prepare PX4 code

To build the PX4 code, use:

```sh
DONT_RUN=1 make px4_sitl gazebo mavsdk_tests
```

### Run all tests

To run all tests, use:

```sh
test/mavsdk_tests/mavsdk_test_runner.py --speed-factor 20 --gui
```

To see all possible command line arguments, check out:

```sh
test/mavsdk_tests/mavsdk_test_runner.py -h
```
