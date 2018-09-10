# 파라미터 & 설정

PX4 플랫폼은 파라미터 서브시스템(float과 int32_t 값의 일반 테이블)과 텍스트 파일(믹서와 startup 스크립트용)을 사용해서 설정을 저장합니다.

[시스템 startup](../concept/system_startup.md)과 [airframe 설정](../airframes/adding_a_new_frame.md) 동작 방법은 다른 페이지에서 설명합니다. 이 섹션에서는 파라미터 서브시스템에 대해서 상세히 알아봅니다.

## 커맨드 라인 사용법

PX4 [시스템 콘솔](../debug/system_console.md)은 ```param``` 도구를 제공해서 파리미터를 설정하고 그 값을 읽을 수 있으며 파일로 저장하거나 export가 가능합니다.

### 파라미터 읽기/설정하기

모든 시스템 파라미터의 목록을 보여주는 명령:

```sh
param show
```

와일드카드를 사용하면 파라미터 일부 이름으로 찾기가 가능:

```sh
nsh> param show RC_MAP_A*
Symbols: x = used, + = saved, * = unsaved
x   RC_MAP_AUX1 [359,498] : 0
x   RC_MAP_AUX2 [360,499] : 0
x   RC_MAP_AUX3 [361,500] : 0
x   RC_MAP_ACRO_SW [375,514] : 0

 723 parameters total, 532 used.
```

### 파라미터 Exporting 및 Loading

표준 save 명령은 디폴트 파일에 파라미터를 저장:

```sh
param save
```

인자를 제공하는 경우, 새로운 위치에 파라미터를 저장:

```sh
param save /fs/microsd/vtol_param_backup
```

파라미터를 로드하는데 2가지 명령이 있습니다: ```param load```는 파일을 로드하고 현재 파라미터를 이 파일에 있는 것으로 대체하여 결국 1:1 복사가 일어납니다. ```param import```는 디폴트에서 변경된 파라미터 값만 변경합니다. 이것은 초기에 보드를 칼리브레이션하는 경우 유용하며 시스템 설정의 나머지 부분을 덮어쓰기 하지 않고 칼리브레이션 데이터를 가져오는 것이 가능합니다.

현재 파라미터 덮어쓰기:

```sh
param load /fs/microsd/vtol_param_backup
```

현재 파라미터를 저장된 파라미터와 합치기(디폴트가 아닌 저장된 값이 우선):

```sh
param import /fs/microsd/vtol_param_backup
```

## C / C++ API

파라미터 값에 접근하는데 사용할 수 있는 C와 별도 C++ API가 있습니다.

> **Todo** C / C++ API 파라미터 논의.


<div class="host-code"></div>

```C
int32_t param = 0;
param_get(param_find("PARAM_NAME"), &param);
```

## 파라미터 메타 데이터

PX4는 파라미터가 사용자에게 표현되는 것은 확장 파라미터 메타 데이터 시스템을 사용합니다. 제대로된 메타 데이터는 그라운드 스테이션의 UX에 아주 중요합니다.

전형적인 파라미터 메타데이터 섹션은 다음과 같은 형태:

```C++
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

각 라인은 이렇게 사용:

```C++
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
 * @reboot_required true <add this if changing the param requires a system restart>
 * @group <a title for parameters which form a group>
 */
PARAM_DEFINE_FLOAT(MC_PITCH_P, 6.5f);
```
