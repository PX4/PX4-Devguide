# Jenkins CI

Jenkins continuous integration server on [SITL01](http://sitl01.dronetest.io/) is used to automatically run integration tests against PX4 SITL.

## Overview

  * Involved components: Jenkins, Docker, PX4 POSIX SITL
  * Tests run inside [Docker Containers](advanced-docker.md)
  * Jenkins executes 2 jobs: one to check each PR against master, and the other to check every push on master

## Test Execution

Jenkins uses [run_container.bash](https://github.com/PX4/Firmware/blob/master/integrationtests/run_container.bash) to start the container which in turn executes [run_tests.bash](https://github.com/PX4/Firmware/blob/master/integrationtests/run_tests.bash) to compile and run the tests.

If Docker is installed the same method can be used locally:

```sh
cd <directory_where_firmware_is_cloned>
sudo WORKSPACE=$(pwd) ./Firmware/integrationtests/run_container.bash
```

## Server Setup

### Installation

See setup [script/log](https://github.com/PX4/containers/tree/master/scripts/jenkins) for details on how Jenkins got installed and maintained.

### Configuration

  * Jenkins security enabled
  * Installed plugins
    * github
    * github pull request builder
    * embeddable build status plugin
    * s3 plugin
    * notification plugin
    * collapsing console sections
    * postbuildscript
