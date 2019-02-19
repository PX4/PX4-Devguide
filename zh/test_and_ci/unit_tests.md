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
8. 在 **test_ [description].cpp** 的底部声明测试。
    
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

9. 在 [tests_main.h](https://github.com/PX4/Firmware/blob/master/src/systemcmds/tests/tests_main.h) 中定义新测试：
    
    ```cpp
    extern int test_[description](int argc, char *argv[]);
    ```

10. 在 [tests_main.c](https://github.com/PX4/Firmware/blob/master/src/systemcmds/tests/tests_main.c) 中添加描述名称，测试功能和选项：
    
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
    
    1. 将测试`test_ [desciption].cpp`添加到 [CMakeLists.txt]（https://github.com/PX4/Firmware/blob/master/src/systemcmds/tests/CMakeLists.txt）。
    
    
    ## 在本地计算机上进行测试
    
    以下命令足以启动运行 PX4 posix 端口的最小新 shell。
    
    ```bash
    make px4_sitl_shell none
    

然后可以将 shell 用于测试，例如：执行单元测试：

```bash
pxh> tests [description]
```

或者，也可以直接从 bash 运行完整的单元测试：

```bash
make tests
```

要查看 px4 shell 中可用测试的完整列表，请执行以下操作：

```bash
pxh> tests help
```