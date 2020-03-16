# Integration Testing using MAVSDK

PX4 can be tested end to end to using integration tests based on [MAVSDK](https://mavsdk.mavlink.io).

The tests are primarily developed against SITL for now and run in continuous integration (CI).
However, they are meant to generalize to real tests eventually.

## Install the MAVSDK C++ Library

The tests need the MAVSDK C++ library installed system-wide (e.g. in `/usr/lib` or `/usr/local/lib`).

Install either from binaries or source:
- [MAVSDK > Installation > C++](https://mavsdk.mavlink.io/develop/en/getting_started/installation.html#cpp): Install as a prebuilt library on supported platforms (recommended)
- [MAVSDK > Contributing > Building from Source](https://mavsdk.mavlink.io/develop/en/contributing/build.html#build_sdk_cpp): Build  C++ library from source.


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
