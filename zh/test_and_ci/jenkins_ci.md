# Jenkins CI

[ci.px4.io](http://ci.px4.io/) 上的 Jenkins 持续集成服务器用于自动运行针对 PX4 SITL 的集成测试。

## 概述

- 涉及的组件：Jenkins，Docker，PX4 POSIX SITL
- 测试在 [Docker Containers](../test_and_ci/docker.md) 内运行
- Jenkins 执行了 2 个工作：一个用于检查每个 PR 与主控，另一个用于检查主控上的每次推送

## Test Execution

Jenkins uses [run_container.bash](https://github.com/PX4/Firmware/blob/master/integrationtests/run_container.bash) to start the container which in turn executes [run_tests.bash](https://github.com/PX4/Firmware/blob/master/integrationtests/run_tests.bash) to compile and run the tests.

If Docker is installed the same method can be used locally:

```sh
cd <directory_where_firmware_is_cloned>
sudo WORKSPACE=$(pwd) ./Firmware/integrationtests/run_container.bash
```

## 服务器配置

### 安装

See setup [script/log](https://github.com/PX4/containers/tree/master/scripts/jenkins) for details on how Jenkins got installed and maintained.

### 配置

- Jenkins security enabled
- Installed plugins 
    - github
    - github pull request builder
    - embeddable build status plugin
    - s3 plugin
    - notification plugin
    - collapsing console sections
    - postbuildscript