# 매개변수와 설정

PX4는 설정 값을 저장하는 수단으로 *매개변수 하위 체계* (`float`형과 `int32_t`형 값의 단순 집합)와 텍스트 파일(믹서, 시작 스크립트용)을 사용합니다.

이 장에서는 *매개변수* 하위 시스템을 자세하게 다루도록 하겠습니다. 매개변수를 어떻게 살펴보고, 저장하고, 불러오고, 지정하는지를 다룹니다.

> **Note** [시스템 시작](../concept/system_startup.md)과 [에어프레임 설정](../airframes/adding_a_new_frame.md) 작업 방법은 다른 페이지에서 자세하게 언급합니다.

## 명령행 사용법

PX4 [시스템 콘솔](../debug/system_console.md)에서는 [매개변수](../middleware/modules_command.md#param)값을 파일로(부터) 설정, 불러오기, 저장, 내보내기, 복원 처리하는 도구를 제공합니다.

### 매개변수 값 가져오고 설정하기

`param show` 명령은 전체 시스템 매개변수 값 목록을 보여줍니다:

```sh
param show
```

좀 더 원하는 부분을 선택할 경우, 매개변수 이름 일부 대신 와일드 카드 문자 "*"를 사용할 수 있습니다:

```sh
nsh> param show RC_MAP_A*
Symbols: x = used, + = saved, * = unsaved
x   RC_MAP_AUX1 [359,498] : 0
x   RC_MAP_AUX2 [360,499] : 0
x   RC_MAP_AUX3 [361,500] : 0
x   RC_MAP_ACRO_SW [375,514] : 0

 723 parameters total, 532 used.
```

`-c` 플래그를 사용하여 (기본값으로부터) 값이 바뀐 모든 매개변수를 확인할 수 있습니다:

```sh
param show -c
```

You can use `param show-for-airframe` to show all parameters that have changed from their defaults for just the current airframe's definition file (and defaults it imports).

### 매개변수 값 불러오고 내보내기

You can save any parameters that have been *touched* since all parameters were last reset to their firmware-defined defaults (this includes any parameters that have been changed, even if they have been changed back to their default).

The standard `param save` command will store the parameters in the current default file:

```sh
param save
```

If provided with an argument, it will store the parameters instead to this new location:

```sh
param save /fs/microsd/vtol_param_backup
```

There are two different commands to *load* parameters:

- 우선 `param load` 명령은 모든 매개변수 값을 기본값으로 초기화하며, 파일에 저장한 어떤 값이든 덮어씁니다.
- `param import`는 파일에서 가져온 매개변수 값을 덮어쓰기만 하고, 결과를 저장합니다(예: `param save` 명령 호출과 동일한 결과).

The `load` effectively resets the parameters to the state when the parameters were saved (we say "effectively" because any parameters saved in the file will be updated, but other parameters may have different firmware-defined default values than when the parameters file was created).

By contrast, `import` merges the parameters in the file with the current state of the vehicle. This can be used, for example, to just import a parameter file containing calibration data, without overwriting the rest of the system configuration.

Examples for both cases are shown below:

```sh
# 파일을 저장하고 나면 매개변수 값 초기화
param load /fs/microsd/vtol_param_backup
# 추가로 매개변수 값 저장 (불러온다고 해서 자동으로 끝나지는 않음)
param save
```

```sh
# 현재 매개변수 값 목록에 저장한 매개변수 값 병합
param import /fs/microsd/vtol_param_backup  
```

## 매개변수 이름

Parameter names must be no more than 16 ASCII characters.

By convention, every parameter in a group should share the same (meaningful) string prefix followed by an underscore, and `MC_` and `FW_` are used for parameters related specifically to Multicopter or Fixed wing systems. This convention is not enforced.

The name must match in both code and [parameter metadata](#parameter_metadata) to correctly associate the parameter with its metadata (including default value in Firmware).

## C / C++ API

There are separate C and C++ APIs that can be used to access parameter values from within PX4 modules and drivers.

One important difference between the APIs is that the C++ version has a more efficient standardized mechanism to synchronize with changes to parameter values (i.e. from a GCS).

Synchronization is important because a parameter can be changed to another value at any time. Your code should *always* use the current value from the parameter store. If getting the latest version is not possible, then a reboot will be required after the parameter is changed (set this requirement using the `@reboot_required` metadata).

In addition, the C++ version has also better type-safety and less overhead in terms of RAM. The drawback is that the parameter name must be known at compile-time, while the C API can take a dynamically created name as a string.

### C++ API

The C++ API provides macros to declare parameters as *class attributes*. You add some "boilerplate" code to regularly listen for changes in the [uORB Topic](../middleware/uorb.md) associated with *any* parameter update. Framework code then (invisibly) handles tracking uORB messages that affect your parameter attributes and keeping them in sync. In the rest of the code you can just use the defined parameter attributes and they will always be up to date!

First include **px4_platform_common/module_params.h** in the class header for your module or driver (to get the `DEFINE_PARAMETERS` macro):

```cpp
#include <px4_platform_common/module_params.h>
```

Derive your class from `ModuleParams`, and use `DEFINE_PARAMETERS` to specify a list of parameters and their associated parameter attributes. The names of the parameters must be the same as their parameter metadata definitions.

```cpp
class MyModule : ..., public ModuleParams
{
public:
    ...

private:

    /**
     * 필요한 경우 매개변수 값을 확인하고 새 값을 설정한다.
     * @param parameter_update_sub uorb subscription to parameter_update
     */
    void parameters_update(int parameter_update_sub, bool force = false);

    DEFINE_PARAMETERS(
        (ParamInt<px4::params::SYS_AUTOSTART>) _sys_autostart,   /**< example parameter */
        (ParamFloat<px4::params::ATT_BIAS_MAX>) _att_bias_max  /**< another parameter */
    )
};
```

Update the cpp file with boilerplate to check for the uORB message related to parameter updates.

First include the header to access the uORB parameter_update message:

```cpp
#include <uORB/topics/parameter_update.h>
```

Subscribe to the update message when the module/driver starts and un-subscribe when it is stopped. `parameter_update_sub` returned by `orb_subscribe()` is a handle we can use to refer to this particular subscription.

```cpp
# parameter_update 메시지 추가 준비
int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
...
# parameter_update 메시지 추가 해제
orb_unsubscribe(parameter_update_sub);
```

Call `parameters_update(parameter_update_sub);` periodically in code to check if there has been an update (this is boilerplate):

```cpp
void Module::parameters_update(int parameter_update_sub, bool force)
{
    bool updated;
    struct parameter_update_s param_upd;

    // 매개변수 값 변경 확인
    orb_check(parameter_update_sub, &updated);

    // 바뀐 매개변수 값이 있을 경우 param_upd로 복사
    if (updated) {
        orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_upd);
    }

    if (force || updated) {
        // 바뀐 매개변수 값이 있다면 updateParams() 를 호출,
        // 이 클래스 속성 값을 업데이트해야 하는지 검사(그리고 이행). 
        updateParams();
    }
}
```

In the above method:

- `param_update` uORB 메시지에 *어떤* 업데이트 사항이 있다면 `orb_check()`에서 알려주고 (다만 어떤 매개변수가 영향을 받았는지 정보는 아님) `updated` 부울린 값을 설정합니다.
- "일부" 매개변수를 업데이트했다면 `parameter_update_s` (`param_upd`)에 최신 매개변수 값을 복사합니다
- 그 다음 `ModuleParams::updateParams()` 메서드를 호출합니다. 이 "하부" 에서는 `DEFINE_PARAMETERS` 목록에 있는 특정 매개변수 속성을 업데이트해야 하는지 확인하고, 필요할 경우 진행합니다.
- 이 예제에서는 `Module::parameters_update()` 메서드를 `force=True` 인자 값을 대입하여 호출하지 않습니다. 만약 함수에 넣은 일반 패턴을 설정해야 할 다른 값이 있다면, 초기화를 진행하는 동안 `force=True` 값을 대입하여 1회 호출합니다.

The parameter attributes (`_sys_autostart` and `_att_bias_max` in this case) can then be used to represent the parameters, and will be updated whenever the parameter value changes.

> [어플리케이션/모듈 서식](../apps/module_template.md)에서는 새 방식의 C++ API를 사용하나 [매개변수 메타데이터](#parameter_metadata)는 들어있지 않습니다.

### C API

The C API can be used within both modules and drivers.

First include the parameter API:

```C
#include <parameters/param.h>
```

Then retrieve the parameter and assign it to a variable (here `my_param`), as shown below for `PARAM_NAME`. The variable `my_param` can then be used in your module code.

```C
int32_t my_param = 0;
param_get(param_find("PARAM_NAME"), &my_param);
```

> **Note** `PARAM_NAME`을 매개변수 메타데이터에서 선언했다면 우선 이 기본값을 설정하며, 위 코드에서의 매개변수 검색 호출은 언제든 성공합니다.

`param_find()` is an "expensive" operation, which returns a handle that can be used by `param_get()`. If you're going to read the parameter multiple times, you may cache the handle and use it in `param_get()` when needed

```cpp
# 매개변수 핸들러 획득
param_t my_param_handle = PARAM_INVALID;
my_param_handle = param_find("PARAM_NAME");

# 필요할 때 매개변수 값 요청
int32_t my_param = 0;
param_get(my_param_handle, &my_param);
```

## 매개변수 메타데이터 {#parameter_metadata}

PX4 uses an extensive parameter metadata system to drive the user-facing presentation of parameters, and to set the default value for each parameter in firmware.

> **Tip** 올바른 메타데이터는 지상 관제에 있어 바람직한 사용자 경험을 위해 중요합니다.

Parameter metadata can be stored anywhere in the source tree as either **.c** or **.yaml** parameter definitions (the YAML definition is newer, and more flexible). Typically it is stored alongside its associated module.

The build system extracts the metadata (using `make parameters_metadata`) to build the [parameter reference](../advanced/parameter_reference.md) and the parameter information used by ground stations.

> **Warning** *새* 매개변수 파일을 추가하고 나면, 새 매개변수를 만들기 전 `make clean`을 실행해야합니다(매개변수 파일은 *cmake* 설정 단계의 일부로서 추가하며, 이 명령을 실행하면 cmake 파일을 수장했을 때, 기존의 빌드 파일을 정리합니다).

### C 매개변수 메타데이터 {#c_metadata}

The legacy approach for defining parameter metadata is in a file with extension **.c** (at time of writing this is the approach most commonly used in the source tree).

Parameter metadata sections look like the following examples:

```cpp
/**
 * Pitch P gain
 *
 * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @max 10
 * @decimal 2
 * @increment 0.0005
 * @reboot_required true
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCH_P, 6.5f);
```

```cpp
/**
 * GPS 속도 기반 가속 보상.
 *
 * @group Attitude Q estimator
 * @boolean
 */
PARAM_DEFINE_INT32(ATT_ACC_COMP, 1);
```

The `PARAM_DEFINE_*` macro at the end specifies the type of parameter (`PARAM_DEFINE_FLOAT` or `PARAM_DEFINE_INT32`), the name of the parameter (which must match the name used in code), and the default value in firmware.

The lines in the comment block are all optional, and are primarily used to control display and editing options within a ground station. The purpose of each line is given below (for more detail see [module_schema.yaml](https://github.com/PX4/Firmware/blob/master/validation/module_schema.yaml)).

```cpp
/**
 * <title>
 *
 * <longer description, can be multi-line>
 *
 * @unit <the unit, e.g. m for meters>
 * @min <the minimum sane value. Can be overridden by the user>
 * @max <the maximum sane value. Can be overridden by the user>
 * @decimal <the minimum sane value. Can be overridden by the user>
 * @increment <the "ticks" in which this value will increment in the UI>
 * @reboot_required true <add this if changing the param requires a system restart.>
 * @boolean <add this for integer parameters that represent a boolean value>
 * @group <a title for parameters that form a group>
 */
```

### YAML 메타데이터 {#yaml_metadata}

> **Note** YAML 매개변수 정의를 작성했을 때는 *라이브러리*에서 활용할 수 없습니다.

YAML meta data is intended as a full replacement for the **.c** definitions. It supports all the same metadata, along with new features like multi-instance definitions.

- YAML 매개변수 메타데이터 스키마는 [validation/module_schema.yaml](https://github.com/PX4/Firmware/blob/master/validation/module_schema.yaml)에 있습니다.
- 활용 중인 YAML 정의 예제는 [/src/modules/mavlink/module.yaml](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/module.yaml) MAVLink 매개변수 정의파일에서 찾을 수 있습니다.

#### 다중 인스턴스 (서식화) 메타데이터 {#multi_instance_metadata}

Templated parameter definitions are supported in [YAML parameter definitions](https://github.com/PX4/Firmware/blob/master/validation/module_schema.yaml) (templated parameter code is not supported).

The YAML allows you to define instance numbers in parameter names, descriptions, etc. using `${i}`. For example, below will generate MY_PARAM_1_RATE, MY_PARAM_2_RATE etc.

    MY_PARAM_${i}_RATE:
                description:
                    short: Maximum rate for instance ${i}
    

The following YAML definitions provide the start and end indexes.

- `num_instances` (기본값 1): 생성할 인스턴스 갯수(하나 이상)
- `instance_start` (기본값 0): 첫번재 인스턴스 번호. 0으로 지정하면, `${i}` 값은 0부터 N-1 까지 갑니다.

For a full example see the MAVLink parameter definitions: [/src/modules/mavlink/module.yaml](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/module.yaml)

## 추가 정보

- [매개변수 검색/업데이트](https://docs.px4.io/master/en/advanced_config/parameters.html) (PX4 사용자 안내서)
- [매개변수 참고서](../advanced/parameter_reference.md)