# 외부 모듈

여기서는 PX4 빌드에 외부 모듈을 추가하는 방법에 대해서 설명합니다.

외부 모듈은 내부 모듈과 동일하게 includes를 사용할 수 있으며 uORB를 이용해서 내부 모듈과 통신할 수 있습니다.

## 사용법

- `EXTERNAL_MODULES_LOCATION` Firmware와 동일한 구조를 가지는 디렉토리를 가리켜야 하는 경우(`src`라는 디렉토리를 포함하는)
- 2가지 옵션: 기본 모듈을 외부 디렉토리에 복사(예로 examples/px4_simple_app)하거나 새로운 모듈을 직접 생성
- 모듈 이름바꾸기 (CMakeLists.txt에 `MODULE`을 포함) 혹은 기존 Firmware cmake 빌드 config에서 삭제. 내부 모듈과 충돌을 피하기 위한 방법입니다.
- `$EXTERNAL_MODULES_LOCATION/CMakeLists.txt` 라는 파일에 내용을 채워서 추가:

```
set(config_module_list_external
    modules/<new_module>
    PARENT_SCOPE
    )
```
- `EXTERNAL` 라인을 `px4_add_module` 내부에 `modules/<new_module>/CMakeLists.txt`에 추가합니다. 다음은 예제입니다:

```
px4_add_module(
	MODULE modules__test_app
	MAIN test_app
	STACK_MAIN 2000
	SRCS
		px4_simple_app.c
	DEPENDS
		platforms__common
	EXTERNAL
	)

```
- `make posix EXTERNAL_MODULES_LOCATION=<path>` 실행. 다른 build target도 사용할 수 있지만 build 디렉토리는 없는 상태여야만 합니다. 만약 기존에 있다면 build 폴더에 cmake 변수만 설정해 줄수도 있습니다.
  다음 추가 빌드에서 `EXTERNAL_MODULES_LOCATION`를 더이상 지정할 필요가 없습니다.
