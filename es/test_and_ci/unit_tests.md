# Unit Tests

PX4 provides a simple base [Unittest-class](https://github.com/PX4/Firmware/blob/master/src/include/unit_test.h). Each developer is encouraged to write unit tests in the process of adding a new feature to the PX4 framework.

## Writing a Test

1. Create a new .cpp file within [tests](https://github.com/PX4/Firmware/tree/master/src/systemcmds/tests) with name **test_[description].cpp**. 
2. Within **test_[description].cpp** include the base unittest-class `<unit_test.h>` and all files required to write a test for the new feature. 
3. Within **test_[description].cpp** create a class `[Description]Test` that inherits from `UnitTest`.
4. Within `[Description]Test` class declare the public method `virtual bool run_tests()`.
5. Within `[Description]Test` class declare all private methods required to test the feature in question (`test1()`, `test2()`,...).
6. Within **test_[description].cpp** implement the `run_tests()` method where each test[1,2,...] will be run.
7. Within **test_[description].cpp**, implement the various tests.
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