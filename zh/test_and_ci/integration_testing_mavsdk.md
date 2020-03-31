# 使用 MAVSDK 集成测试

可以使用基于
 MAVSDK </ 0>的集成测试对PX4进行端到端测试。</p> 

目前主要针对SITL开发测试，并在持续集成（CI）中运行。 但是，它们最终旨在推广到实际测试。



## 安装 MAVSDK C++ 库

测试需要将MAVSAK C++库安装到系统目录（如： `/usr/lib` or `/usr/local/lib`）

二进行安装或源码安装：

- [MAVSDK > 安装 > C++](https://mavsdk.mavlink.io/develop/en/getting_started/installation.html#cpp): 以支持的系统上安装预构建库 (推荐)
- [MAVSDK > 贡献 > 通过源码 ](https://mavsdk.mavlink.io/develop/en/contributing/build.html#build_sdk_cpp): 通过源码编译构建C++库。



## 准备 PX4 源码

使用以下命令构建PX4源码：



```sh
DONT_RUN=1 make px4_sitl gazebo mavsdk_tests
```




### 运行所有PX4测试

运行[sitl.json](https://github.com/PX4/Firmware/blob/master/test/mavsdk_tests/configs/sitl.json)中定义的所有SITL测试，运行：



```sh
test/mavsdk_tests/mavsdk_test_runner.py test/mavsdk_tests/configs/sitl.json --speed-factor 10
```


要看所有可用的命令行参数，运行：



```sh
test/mavsdk_tests/mavsdk_test_runner.py -h

用法：mavsdk_test_runner。 y [-h] [--log-dir LOG_DIR] [--speed-factor SPEED_FACTOR] [--trerations ITERATION] [--abort-early] [--gui] [--model MODEL]
                             [--case CASE] [--debugger DEBUGER] [--verbose]
                             config_file

posital 参数：
  config_file JSON 使用的JSON配置文件

optional 参数：
  -h, --help 显示此帮助信息并退出
  --log-dir LOG_DIR 日志文件目录
  --speed-factor SPEED_FACTOR
                        模拟运行的速度因子
  --迭代ITERATION
                        在首次失败的测试中运行所有测试的频率
  --abort-early 中止
  --guide 显示模拟的可视化化
  MODEL 只为一个模型运行测试
  --case CASE 只运行测试一个案例
  --debugger DEBUGER 调试器：callgrind, gdb, lldb
  --verbose 启用更详细的输出
```




## 关于实现的说明

- 使用Python编写的测试运行程序脚本 mavsdk_test_runner.py </ 0>调用这些测试。 该运行程序还启动` px4 </ 0>以及用于SITL测试的Gazebo，并收集这些进程的日志。</p></li>
<li><p spaces-before="0">这个测试运行器是一个C++库 
它包含了：</p>

<ul>
<li>解析参数的 <a href="https://github.com/PX4/Firmware/blob/master/test/mavsdk_tests/test_main.cpp">main</a> 函数。</li>
<li>MAVSDK的抽象称为<a href="https://github.com/PX4/Firmware/blob/master/test/mavsdk_tests/autopilot_tester.h"> autopilot_tester </a>。</li>
<li>The actual tests using the abstraction around MAVSDK as e.g. <a href="https://github.com/PX4/Firmware/blob/master/test/mavsdk_tests/test_multicopter_mission.cpp">test_multicopter_mission.cpp</a>.</li>
<li>The tests use the <a href="https://github.com/catchorg/Catch2">catch2</a> unit testing framework.
The reasons for using this framework are:

<ul>
<li>Asserts (<code>REQUIRE`) which are needed to abort a test can be inside of functions (and not just in the top level test as is [the case with gtest](https://github.com/google/googletest/blob/master/googletest/docs/advanced.md#assertion-placement)).</li> 
  
        - Dependency management is easier because *catch2* can just be included as a header-only library.
      - *Catch2* supports [tags](https://github.com/catchorg/Catch2/blob/master/docs/test-cases-and-sections.md#tags), which allows for flexible composition of tests.</ul></li> </ul></li> </ul> 

Terms used:

- "model": This is the selected Gazebo model, e.g. `iris`.
- "test case": This is a [catch2 test case](https://github.com/catchorg/Catch2/blob/master/docs/test-cases-and-sections.md).
