# 单元测试

PX4 提供了一个简单的基础 [Unittest-class](https://github.com/PX4/Firmware/blob/master/src/include/unit_test.h)。 鼓励每个开发人员在向 PX4 框架添加新功能的过程中编写单元测试。

## 编写测试

1. 在 [tests](https://github.com/PX4/Firmware/tree/master/src/systemcmds/tests) 中创建名为 **test_ [description] .cpp** 的新 .cpp 文件。 
2. 在 **test_[description].cpp** 中包括基本 unittest-class`&lt;unit_test.h&gt;` 以及为新功能编写测试所需的所有文件。 
3. 在 **test_[description].cpp** 中创建一个继承自 `UnitTest` 的类 `[Description]Test`。
4. 在 `[Description]Test` 类中，声明公共方法 `virtual bool run_tests（）`。
5. 在 `[Description]Test` 类中，声明测试相关特征所需的所有私有方法（` test1（）`，` test2（）`，...）。
6. 在 **test_ [description].cpp** 中实现 `run_tests（）` 方法，其中将运行每个测试[1,2，...]。
7. 在 **test_ [description].cpp** 中，实现各种测试。
8. At the bottom within **test_[description].cpp** declare the test.
    
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

9. Within [tests_main.h](https://github.com/PX4/Firmware/blob/master/src/systemcmds/tests/tests_main.h) define the new test:
    
    ```cpp
    extern int test_[description](int argc, char *argv[]);
    ```

10. Within [tests_main.c](https://github.com/PX4/Firmware/blob/master/src/systemcmds/tests/tests_main.c) add description name, test function and option:
    
    ```cpp ... } tests[] = { {... {"[description]", test_[description], OPTION}, ... }

       `OPTION` can be `OPT_NOALLTEST`,`OPT_NOJIGTEST` or `0` and is considered if within px4 shell one of the two commands are called:
    
       ```bash
       pxh> tests all
       ```
       or
    
       ```bash
       pxh> tests jig
       ```
       If a test has option `OPT_NOALLTEST`, then that test will be excluded when calling `tests all`. The same is true for `OPT_NOJITEST` when command `test jig` is called. Option `0` means that the test is never excluded, which is what most developer want to use. 
    
    1. Add the test `test_[desciption].cpp` to the [CMakeLists.txt](https://github.com/PX4/Firmware/blob/master/src/systemcmds/tests/CMakeLists.txt).
    
    
    ## Testing on the local machine
    
    The following command is sufficient to start a minimal new shell with the PX4 posix port running.
    
    ```bash
    make px4_sitl_shell none
    

The shell can then be used to e.g. execute unit tests:

```bash
pxh> tests [description]
```

Alternatively it is also possible to run the complete unit-tests right from bash:

```bash
make tests
```

To see a full list of available tests write within px4 shell:

```bash
pxh> tests help
```