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

### 매개변수 값 불러오고 내보내기

모든 매개변수 값을 펌웨어 지정 기본값으로 초기화한 이래로 *값을 바꾼*적이 있는 임의의 매개변수를 저장할 수 있습니다.

`param save` 표준 명령은 현재 기본 파일에 매개변수 값을 저장합니다:

```sh
param save
```

인자 값을 제시했다면 새 위치에 매개변수 값을 저장합니다:

```sh
param save /fs/microsd/vtol_param_backup
```

매개변수를 *불러오는(load)* 명령에는 두가지가 있습니다:

- 우선 `param load` 명령은 모든 매개변수 값을 기본값으로 초기화하며, 파일에 저장한 어떤 값이든 덮어씁니다.
- `param import`는 파일에서 가져온 매개변수 값을 덮어쓰기만 하고, 결과를 저장합니다(예: `param save` 명령 호출과 동일한 결과).

`load` 명령은 "실질적"으로 매개변수 값을 저장했을 때의 상태로 초기화합니다("실질적"이란 표현을 쓴 이유는 파일에 저장한 어떤 매개변수 값이든 업데이트하겠지만, 다른 매개변수는 파일을 만들었을 때의 매개변수 값과는 다른 펌웨어 지정 기본 값을 가집니다).

반면에, `import` 명령은 비행체의 현재 상태 값과 파일의 매개변수 값을 병합합니다. 이를테면 시스템 설정의 나머지 부분을 덮어쓰기 하지 않고 보정 값을 동반하여 매개변수 값 파일을 내용을 가져올 때 사용할 수 있습니다.

아래 예제에 두가지 경우를 모두 보여드립니다:

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

매개변수 이름은 ASCII 문자 16개를 넘어서는 안됩니다.

관례에 따르면, 그룹의 모든 매개변수는 밑줄 문자가 뒤따라오는 동일한(의미를 가진) 문자열을 공유하며, 다중 프로펠러 항공기 또는 고정익 항공기의 여부에 따라 `MC_`와 `FW_`를 매개변수 이름에 활용합니다. 이 관례는 강제 사항이 아닙니다.

이름은 매개변수와 (펌웨어 기본값이 들어있는) 메타데이터가 올바르게 붙도록 코드와 [매개변수 메타데이터](#parameter_metadata)에 일치해야합니다.

## C / C++ API

PX4 모듈과 드라이버에서 매개변수 값에 접근하는 용도로 C와 C++언어로 활용할 수 있는 개별 API가 있습니다.

API간 중요한 차이점이 있다면, C++ 버전은 매개변수 값을 바꿀 때 동기화하는 표준 매커니즘이 더욱 효율적입니다(예: GCS에서 업데이트).

매개변수 값이 다른 값으로 언제든 바뀔 수 있으므로 동기화는 중요합니다. 코드는 매개변수 저장소에 *항상* 현재 값을 사용해야합니다. 최근 버전을 가져올 수 없다면, 매개변수 값을 바꾼 후 다시 부팅해야 합니다(`@reboot_required` 메타데이터로 필수 여부 설정).

게다가, C++ 버전은 자료형 관리에 있어 더욱 안전하며 RAM 사용 부하량이 적습니다. 문제점이 있다면, C API는 동적으로 문자열로 만든 이름을 취할 수 있지만, C++ API에서는 컴파일 시간에 매개변수 이름을 밝혀야합니다.

### C++ API

C++ API에서는 *클래스 속성*으로 매개변수를 선언하는 매크로를 제공합니다. *임의의* 매개변수 업데이트와 관련된 [uORB 토픽](../middleware/uorb.md)의 변경을 주기적으로 확인하는 상용구 코드를 추가합니다. Framework code then (invisibly) handles tracking uORB messages that affect your parameter attributes and keeping them in sync. In the rest of the code you can just use the defined parameter attributes and they will always be up to date!

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

상용구 코드(boilerplate)로 cpp 파일을 업데이트하여 매개변수 업데이트와 관련 있는 uORB 메시지를 확인하도록 합니다.

우선 헤더를 포함하여 uORB parameter_update 메시지에 접근하게 합니다:

```cpp
#include <uORB/topics/parameter_update.h>
```

모듈 또는 드라이버를 시작할 때 메시지 업데이트 과정에 참여(subscribe)하며, 과정이 끝나면 해제합니다. `orb_subscribe()`에서 반환하는 `parameter_update_sub` 값은 메시지 업데이트 과정 참여시 참고할 핸들입니다.

```cpp
# parameter_update 메시지 추가 준비
int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
...
# parameter_update 메시지 추가 해제
orb_unsubscribe(parameter_update_sub);
```

`parameters_update(parameter_update_sub);`를 주기적으로 호출하여 업데이트한 매개변수가 있는지 확인하십시오:

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

위 메서드에서:

- `param_update` uORB 메시지에 *어떤* 업데이트 사항이 있다면 `orb_check()`에서 알려주고 (다만 어떤 매개변수가 영향을 받았는지 정보는 아님) `updated` 부울린 값을 설정합니다.
- "일부" 매개변수를 업데이트했다면 `parameter_update_s` (`param_upd`)에 최신 매개변수 값을 복사합니다
- 그 다음 `ModuleParams::updateParams()` 메서드를 호출합니다. 이 "하부" 에서는 `DEFINE_PARAMETERS` 목록에 있는 특정 매개변수 속성을 업데이트해야 하는지 확인하고, 필요할 경우 진행합니다.
- 이 예제에서는 `Module::parameters_update()` 메서드를 `force=True` 인자 값을 대입하여 호출하지 않습니다. 만약 함수에 넣은 일반 패턴을 설정해야 할 다른 값이 있다면, 초기화를 진행하는 동안 `force=True` 값을 대입하여 1회 호출합니다.

매개변수 속성(이 경우, `_sys_autostart` 와 `_att_bias_max`)은 매개변수를 대신할 목적으로 활용할 수 있으며, 매개변수 값이 바뀔 때마다 업데이트합니다.

> **Tip** The [Application/Module Template](../apps/module_template.md) uses the new-style C++ API but does not include [parameter metadata](#parameter_metadata).

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

> **Note** If `PARAM_NAME` was declared in parameter metadata then its default value will be set, and the above call to find the parameter should always succeed.

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

> **Tip** Correct meta data is critical for good user experience in a ground station.

Parameter metadata can be stored anywhere in the source tree as either **.c** or **.yaml** parameter definitions (the YAML definition is newer, and more flexible). Typically it is stored alongside its associated module.

The build system extracts the metadata (using `make parameters_metadata`) to build the [parameter reference](../advanced/parameter_reference.md) and the parameter information used by ground stations.

> **Warning** After adding a *new* parameter file you should call `make clean` before building to generate the new parameters (parameter files are added as part of the *cmake* configure step, which happens for clean builds and if a cmake file is modified).

### c Parameter Metadata {#c_metadata}

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

YAML 메타데이터는 **.c**의 정의를 완전히 대체할 용도로 존재합니다. 모든 동일한 메타데이터를 지원하며, 다중 인스턴스 정의와 같은 새 기능도 있습니다.

- YAML 매개변수 메타데이터 스키마는 [validation/module_schema.yaml](https://github.com/PX4/Firmware/blob/master/validation/module_schema.yaml)에 있습니다.
- 활용 중인 YAML 정의 예제는 [/src/modules/mavlink/module.yaml](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/module.yaml) MAVLink 매개변수 정의파일에서 찾을 수 있습니다.

#### 다중 인스턴스 (서식화) 메타데이터 {#multi_instance_metadata}

Templated parameter definitions are supported in [YAML parameter definitions](https://github.com/PX4/Firmware/blob/master/validation/module_schema.yaml) (templated parameter code is not supported).

The YAML allows you to define instance numbers in parameter names, descriptions, etc. using `${i}`. For example, below will generate MY_PARAM_1_RATE, MY_PARAM_2_RATE etc.

    MY_PARAM_${i}_RATE:
                description:
                    short: Maximum rate for instance ${i}
    

The following YAML definitions provide the start and end indexes.

- `num_instances` (default 1): Number of instances to generate (>=1)
- `instance_start` (default 0): First instance number. If 0, `${i}` expands to [0, N-1]`.

For a full example see the MAVLink parameter definitions: [/src/modules/mavlink/module.yaml](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/module.yaml)

## 추가 정보

- [매개변수 검색/업데이트](https://docs.px4.io/master/en/advanced_config/parameters.html) (PX4 사용자 안내서)
- [매개변수 참고서](../advanced/parameter_reference.md)