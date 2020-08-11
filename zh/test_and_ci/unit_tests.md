# 单元测试

我们鼓励开发人员在开发的每个模块时编写单元测试，包括添加新功能，修复错误和重构。

PX4提供了以下几种编写单元测试的方法：

1. 使用[Google Test](https://github.com/google/googletest/blob/master/googletest/docs/primer.md) ("GTest")进行单元测试 - 最小程度的内部依赖
2. 使用GTest的功能测试 - 依赖parameters和 uORB和消息
3. 软件在环(SITL)单元测试。 这些测试需要运行在完整的SITL环境中， 运行起来更慢，更难调试，所以建议尽可能使用GTest代替。

## 写一个GTest单元测试

**Tip**: 一般来说，如果你需要访问更高级的GTest工具、SITL的数据结构或者需要链接`parameters` 或 `uorb` 库，你应改用功能测试。

创建新的单元测试步骤如下：

1. 单元测试分成三个部分：设置、运行、检查结果。 每个单元测试都应该测试一个特定行为或设置案例，如果测试失败，则很明显你的测试代码有错误。 请尽可能遵循这些标准。
2. 复制示例单元测试 [AttitudeControlTest](https://github.com/PX4/Firmware/blob/master/src/modules/mc_att_control/AttitudeControl/AttitudeControlTest.cpp) 到你测试目录，并重命名。
3. 将新文件到该目录的`CMakeLists.txt`文件中。 文件看起来像`px4_add_unit_gtest(SRC MyNewUnitTest.cpp LINKLIBS <library_to_be_tested>)`
4. 添加你想要的测试功能。 这包括了添加所需的头文件、新测试(每个测试都应该有单独的名称)，并加入相关逻辑，运行测试代码并验证其行为是否符合预期。
5. 如果需要添加新的依赖库，只要在如上所说的CMakeLists文件中`LINKLIBS`后面加入库的名字。

可以通用 `make tests`命令来运行所有测试，然后在 `build/px4_sitl_test/unit-MyNewUnit`目录中找到二进行制文件。 也可以直接通过调试器中运行。

## 写一个GTest功能测试

当测试或测试的组件依赖参数、uORB 消息、或更高级的GTest功能的时候，应当使用GTest功能测试。 此外，功能测试可以包含STL数据结构的本地用法（要小心平台之间的差异，如 macOS 和 Linux)

创建一个新的功能测试步骤如下：

1. 一般来说（与单元测试类似）功能测试应分为三个部分：设置，运行，检查结果。 每个测试都应该包括一个特定行为或是设置案例，如果测试失败，则很明显有错误。 请尽可能遵循这些标准。
2. 复制示例功能测试 [ParameterTest](https://github.com/PX4/Firmware/blob/master/src/lib/parameters/ParameterTest.cpp) 到你测试目录，并重命名。
3. 将ParameterTest 重命名为更符合你正在测试的代码功能。
4. 将新文件添加到目录里面的`CMakeLists.txt`。 文件内容看起来像 `px4_add_functional_gtest(SRC MyNewFunctionalTest.cpp LINKLIBS <library_to_be_tested>)`
5. 添加你想要的测试功能。 这包括了，添加特定的头文件、新测试（每个测试都应该使用不同的命名），并设置相关逻辑，运行测试代码并验证是否符合预期。
6. 如果需要添加新的依赖库，只要在如上所说的CMakeLists文件中LINKLIBS后面加入库的名字。

可以通用`make tests`命令来运行所有测试，然后在 `build/px4_sitl_test/functional-MyNewFunctional`目录中找到二进行制文件。 也可以直接通过调试器中运行。 可以直接在调试器中运行，但是当每个可执行文件只有一个测试的时候请谨慎使用[--gtest_filter=<regex>](https://github.com/google/googletest/blob/master/googletest/docs/advanced.md#running-a-subset-of-the-tests)参数，因为uORB和参数库的某些部分清理的时候不太完美，如果多次设置，可能会导致不确定的行为。

## 写一个软件在环（SITL）单元测试

当需要所有的飞行控制组件：驱动、时间或者更多时，应该SITL单元测试。 这些测试运行较慢(每个模块至少1秒+)，同时难以测试，所以仅在必要时使用它们。

创建一个新的SITL单元测试步骤如下：

1. 检查示例 [Unittest-class](https://github.com/PX4/Firmware/blob/master/src/include/unit_test.h)。
2. 在 [tests](https://github.com/PX4/Firmware/tree/master/src/systemcmds/tests) 中创建名为 **test_ [description] .cpp** 的新 .cpp 文件。
3. 在 **test_[description].cpp** 中包括基本 unittest-class`<unit_test.h>` 以及为新功能编写测试所需的所有文件。
4. 在 **test_[description].cpp** 中创建一个继承自 `UnitTest` 的类 `[Description]Test`。
5. 在 `[Description]Test` 类中，声明公共方法 `virtual bool run_tests()`。
6. 在 `[Description]Test` 类中，声明测试相关特征所需的所有私有方法（` test1()`，` test2()`，...）。
7. 在 **test_ [description].cpp** 中实现 `run_tests()` 方法，其中将运行每个测试[1,2，...]。
8. 在 **test_ [description].cpp** 中，实现各种测试。
9. 在 **test_ [description].cpp** 的底部声明测试。
    
    ```cpp
    ut_declare_test_c(test_[description], [Description]Test)
    ```
    
    下面是一个模板：
    
    ```cpp
    #include <unit_test.h>
    #include "[new feature].h"
    ...
    
    class [Description]Test : public UnitTest
    {
    public:
       virtual bool run_tests();
    
    private:
       bool test1();
       bool test2();
       ...
    };
    
    bool [Description]Test::run_tests()
    {
       ut_run_test(test1)
       ut_run_test(test2)
       ...
    
       return (_tests_failed == 0);
    }
    
    bool [Description]Test::test1()
    {
       ut_[name of one of the unit test functions](...
       ut_[name of one of the unit test functions](...
       ...
    
       return true;
    }
    
    bool [Description]Test::test2()
    {
       ut_[name of one of the unit test functions](...
       ut_[name of one of the unit test functions](...
       ...
    
       return true;
    }
    ...
    
    ut_declare_test_c(test_[description], [Description]Test)
    ```
    
    注意，`ut_[name of one of the unit test functions]` 对应于 [unit_test.h ](https://github.com/PX4/Firmware/blob/master/src/include/unit_test.h) 中定义的单元测试函数之一。

10. 在 [tests_main.h](https://github.com/PX4/Firmware/blob/master/src/systemcmds/tests/tests_main.h) 中定义新测试：
    
    ```cpp
    extern int test_[description](int argc, char *argv[]);
    ```

11. 在 [tests_main.c](https://github.com/PX4/Firmware/blob/master/src/systemcmds/tests/tests_main.c) 中添加描述名称，测试功能和选项：
    
    ```cpp ... } tests[] = { {... {"[description]", test_[description], OPTION}, ... }

       `OPTION` can be `OPT_NOALLTEST`,`OPT_NOJIGTEST` or `0` and is considered if within px4 shell one of the two commands are called:
    
       ```bash
       pxh> tests all
       ```
       or
    
       ```bash
       pxh> tests jig
       ```
       If a test has option `OPT_NOALLTEST`, then that test will be excluded when calling `tests all`. The same is true for `OPT_NOJITEST` when command `test jig` is called. 选项“0”表示从不排除测试，这是大多数开发人员想要使用的。
    
    1. 将测试`test_ [description].cpp`添加到 [CMakeLists.txt]（https://github.com/PX4/Firmware/blob/master/src/systemcmds/tests/CMakeLists.txt）。
    
    
    ## 本地机器上的测试
    
    运行完整的 GTest 单元测试列表 GTest 功能测试和 SITL 单元测试脚本：
    
    ```bash
    make tests
    

单独的 GTest 测试二进制文件处于`build/px4_sitl_test/` 目录中，可以直接在大多数IDE的调试器中运行。

使用以下命令对ctest名称使用正则表达式对要运行的测试子集进行筛选：

```bash
make tests TESTFILTER=<filter expression>
```

例如：

- `make tests TESTFILTER=unit` only run GTest unit tests
- `make tests TESTFILTER=sitl` only run simulation tests
- `make tests TESTFILTER=Attitude` only run the `AttitudeControl` test