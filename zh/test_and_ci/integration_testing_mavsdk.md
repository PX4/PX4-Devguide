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

- 使用Python编写的测试运行程序脚本 mavsdk_test_runner.py </ 0>调用这些测试。 该运行程序还启动 px4 </ 0>以及用于SITL测试的Gazebo，并收集这些进程的日志。</p></li>
<li><p spaces-before="0">这个测试运行器是一个C++库 
它包含了：</p>

<ul>
<li>解析参数的 <a href="https://github.com/PX4/Firmware/blob/master/test/mavsdk_tests/test_main.cpp">main</a> 函数。</li>
<li>MAVSDK的抽象称为<a href="https://github.com/PX4/Firmware/blob/master/test/mavsdk_tests/autopilot_tester.h"> autopilot_tester </a>。</li>
<li>使用围绕MAVSDK的抽象的实际测试，例如 <a href="https://github.com/PX4/Firmware/blob/master/test/mavsdk_tests/test_multicopter_mission.cpp"> test_multicopter_mission.cpp </a>。</li>
<li>测试使用<a href="https://github.com/catchorg/Catch2">catch2</a>单元测试框架。
使用这个框架的原因如下：

<ul>
<li>终止测试所需的断言（<code> REQUIRE </ 0>）可以位于函数内部（而不仅仅是顶层，如<a href="https://github.com/google/googletest/blob/master/googletest/docs/advanced.md#assertion-placement"> gtest </a>测试所示）。</li>
<li>依赖关系管理比较容易，因为<em x-id="3"> catch2 </em>可以只作为头文件库包含在内。</li>
<li><em x-id="3"> Catch2 </em>支持<a href="https://github.com/catchorg/Catch2/blob/master/docs/test-cases-and-sections.md#tags"> tags </a>，从而可以灵活地组成测试。</li>
</ul></li>
</ul></li>
</ul>

<p spaces-before="0">使用的术语：</p>

<ul>
<li>“model”：这是选定的Gazebo模型，例如 <code> iris </ 0>。</li>
<li>"test case": 这是 <a href="https://github.com/catchorg/Catch2/blob/master/docs/test-cases-and-sections.md">catch2 测试用例</a>。</li>
</ul>
