# 단위 테스트

개발자 여러분은 개발 새 기능 추가, 버그 수정, 리팩토링 등 전 영역에 걸쳐 단위 테스트를 작성하시는게 좋습니다.

PX4에서는 단위 테스트 작성에 필요한 몇가지 수단을 제공합니다:

1. [구글 테스트](https://github.com/google/googletest/blob/master/googletest/docs/primer.md) ("GTest")를 통한 단위 테스트 - 최소한의 내부 전용 의존 요소를 시험
2. GTest로의 기능 시험 - 매개변수와 uORB 메세지에 따른 시험
3. SITL 단위 테스트. 완전한 SITL 실행에 필요한 테스트입니다. 이 테스트는 실행하기에 매우 느리거나 디버깅하기 어려운 부분입니다. 따라서 가능하면 GTest를 활용하시는게 좋습니다.

## GTest 단위 테스트 작성

**Tip**: 보통, 고급 GTest 유틸리티, STL 데이터 구조에 접근해야 하거나 `매개변수` 또는 `uORB` 라이브러리로의 연결이 필요하다면, 기능 테스트를 대신 활용해야합니다.

새 단위 테스트의 작성 절차는 다음과 같습니다:

1. 단위 테스트는 설치, 실행, 결과 검사 세 부분으로 정리해야 합니다. 각 테스트에서는 매우 극히 일부의 동작을 시험하거나 설정 조건을 시험하면, 테스트에 실패했을 경우 어떤 부분에서 문제가 있는지 명백하게 드러납니다. 가능하면 이 표준을 따라주십시오.
2. [AttitudeControlTest](https://github.com/PX4/Firmware/blob/master/src/modules/mc_att_control/AttitudeControl/AttitudeControlTest.cpp) 예제 단위 테스트를 시험할 코드가 있는 디렉터리로 복사하고 이름을 바꾸십시오.
3. 해당 디렉터리의 `CMakeLists.txt`에 새 파일을 추가하십시오. `px4_add_unit_gtest(SRC MyNewUnitTest.cpp LINKLIBS <library_to_be_tested>)`와 같아야 합니다.
4. 원하는 시험 기능을 추가하십시오. 특정 테스트를 수행하려면 헤더 파일 추가가 필요하며, 새 테스트 추가(제각각의 이름을 지님), 설정 로직 배치, 시험할 코드 실행, 결과 검증을 기대대로 수행합니다.
5. 추가 라이브러리 의존 요소가 필요하다면, 위에서와 같이 CMakeLists의 `LINKLIBS` 다음에 추가해야합니다.

`make tests`를 실행하여 시험을 진행할 수 있으며, 이 과정 후 `build/px4_sitl_test/unit-MyNewUnit` 위치에서 바이너리를 찾을 수 있습니다. 디버거에서 바로 실행할 수 있습니다.

## GTest 기능 테스트 작성

GTest 기능 시험은 매개변수, uORB 메세지, 고급 GTest 기능에 따라 테스트할 테스트 단위 또는 구성 요소가 있을 때 활용해야합니다. 게다가, 기능 테스트 과정에서 자체 STL 데이터 구조를 사용할 수 있습니다(플랫폼간 차이에 유의해야 함. 예: 맥 OS와 리눅스).

새 기능 테스트의 작성 절차는 다음과 같습니다:

1. In general (and similar to unit tests), functional tests should be arranged in three sections: setup, run, check results. Each test should test one very specific behavior or setup case, so if a test fails it is obvious what is wrong. Please try to follow these standards when possible.
2. Copy and rename the example functional test [ParameterTest](https://github.com/PX4/Firmware/blob/master/src/lib/parameters/ParameterTest.cpp) to the directory the code to be tested is in.
3. Rename the class from ParameterTest to something better representing the code being testing
4. Add the new file to the directory's `CMakeLists.txt`. It should look something like `px4_add_functional_gtest(SRC MyNewFunctionalTest.cpp LINKLIBS <library_to_be_tested>)`
5. Add the desired test functionality. This will mean including the header files required for your specific tests, adding new tests (each with an individual name) and putting the logic for the test setup, running the code to be tested and verifying that it behaves as expected.
6. If additional library dependencies are required, they should also be added to the CMakeLists after the `LINKLIBS` as shown above.

Tests can be run via `make tests`, after which you will find the binary in `build/px4_sitl_test/functional-MyNewFunctional`. It can be run directly in a debugger, however be careful to only run one test per executable invocation using the [--gtest_filter=<regex>](https://github.com/google/googletest/blob/master/googletest/docs/advanced.md#running-a-subset-of-the-tests) arguments, as some parts of the uORB and parameter libraries don't clean themselves up perfectly and may result in undefined behavior if set up multiple times.

## SITL 단위 테스트 작성

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
    
    서식은 아래와 같습니다:
    
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
    
    참고로 `ut_[name of one of the unit test functions]`는 [unit_test.h](https://github.com/PX4/Firmware/blob/master/src/include/unit_test.h)에 지정한 단위 테스트 함수 중 하나에 해당합니다.

10. [tests_main.h](https://github.com/PX4/Firmware/blob/master/src/systemcmds/tests/tests_main.h)에서 새 테스트를 정의하십시오:
    
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
       If a test has option `OPT_NOALLTEST`, then that test will be excluded when calling `tests all`. The same is true for `OPT_NOJITEST` when command `test jig` is called. Option `0` means that the test is never excluded, which is what most developer want to use.
    
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

예를 들어:

- `make tests TESTFILTER=unit` 는 GTest 단위 테스트만 실행합니다
- `make tests TESTFILTER=sitl` 은 모의시험 테스트만 실행합니다
- `make tests TESTFILTER=Attitude` `AttitudeControl` 시험만 실행합니다