# 单元测试

我们鼓励开发人员在开发的每个模块时编写单元测试，包括添加新功能，修复错误和重构。

PX4提供了以下几种编写单元测试的方法：

1. 使用[Google Test](https://github.com/google/googletest/blob/master/googletest/docs/primer.md) ("GTest")进行单元测试 - 最小程度的内部依赖
2. 使用GTest的功能测试 - 依赖parameters和 uORB和消息
3. 软件在环(SITL)单元测试。 这些测试需要运行在完整的SITL环境中， 运行起来更慢，更难调试，所以建议尽可能使用GTest代替。

## 写一个GTest单元测试

**提示**: 一般来说，如果你需要访问更高级的GTest工具、SITL的数据结构或者需要链接`parameters` 或 `uorb` 库，你应改用功能测试。

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

可以通用`make tests`命令来运行所有测试，然后在 `build/px4_sitl_test/functional-MyNewFunctional`目录中找到二进行制文件。 也可以直接通过调试器中运行。 It can be run directly in a debugger, however be careful to only run one test per executable invocation using the [--gtest_filter=<regex>](https://github.com/google/googletest/blob/master/googletest/docs/advanced.md#running-a-subset-of-the-tests) arguments, as some parts of the uORB and parameter libraries don't clean themselves up perfectly and may result in undefined behavior if set up multiple times.

## Writing a SITL Unit Test

SITL unit tests should be used when you specifically need all of the flight controller components - drivers, time, and more. These tests are slower to run (1s+ for each new module), and harder to debug, so in general they should only be used when necessary.

The steps to create new SITL unit tests are as follows:

1. Examine the sample [Unittest-class](https://github.com/PX4/Firmware/blob/master/src/include/unit_test.h).
2. Create a new .cpp file within [tests](https://github.com/PX4/Firmware/tree/master/src/systemcmds/tests) with name **test_[description].cpp**.
3. Within **test_[description].cpp** include the base unittest-class `<unit_test.h>` and all files required to write a test for the new feature.
4. Within **test_[description].cpp** create a class `[Description]Test` that inherits from `UnitTest`.
5. Within `[Description]Test` class declare the public method `virtual bool run_tests()`.
6. Within `[Description]Test` class declare all private methods required to test the feature in question (`test1()`, `test2()`,...).
7. Within **test_[description].cpp** implement the `run_tests()` method where each test[1,2,...] will be run.
8. Within **test_[description].cpp**, implement the various tests.
9. At the bottom within **test_[description].cpp** declare the test.
    
    ```cpp
    ut_declare_test_c(test_[description], [Description]Test)
    ```
    
    Here is a template:
    
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
    
    Note that `ut_[name of one of the unit test functions]` corresponds to one of the unittest functions defined within [unit_test.h](https://github.com/PX4/Firmware/blob/master/src/include/unit_test.h).

10. Within [tests_main.h](https://github.com/PX4/Firmware/blob/master/src/systemcmds/tests/tests_main.h) define the new test:
    
    ```cpp
    extern int test_[description](int argc, char *argv[]);
    ```

11. Within [tests_main.c](https://github.com/PX4/Firmware/blob/master/src/systemcmds/tests/tests_main.c) add description name, test function and option:
    
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
    
    1. Add the test `test_[description].cpp` to the [CMakeLists.txt](https://github.com/PX4/Firmware/blob/master/src/systemcmds/tests/CMakeLists.txt).
    
    
    ## Testing on the local machine
    
    Run the complete list of GTest Unit Tests, GTest Functional Tests and SITL Unit Tests right from bash:
    
    ```bash
    make tests
    

The individual GTest test binaries are in the `build/px4_sitl_test/` directory, and can be run directly in most IDEs' debugger.

Filter to run only a subset of tests using a regular expression for the ctest name with this command:

```bash
make tests TESTFILTER=<filter expression>
```

For example:

- `make tests TESTFILTER=unit` only run GTest unit tests
- `make tests TESTFILTER=sitl` only run simulation tests
- `make tests TESTFILTER=Attitude` only run the `AttitudeControl` test