# 외부 모듈(트리 외부-Out-of-Tree)

외부 모듈에서는 PX4 펌웨어에 추가하려는 상용 모듈을 관리하고 그룹화하는 편의 매커니즘을 개발자에게 제공합니다. 외부 모듈은 내부 모듈과 마찬가지로 uORB를 통해 내부 모듈과 상호 메시지 교환을 할 수 있도록 활용할 수 있습니다.

이 주제에서는 외부("out of tree") 모듈을 PX4 빌드에 추가하는 방법을 설명합니다.

> **Tip** 가능하다면 PX4 프로젝트에 바뀐 내용을 기여해주시기 바랍니다!

## 사용법

외부 모듈을 만들려면:

- 외부 모듈을 모아둘 *외부 디렉터리*를 만드십시오: 
  - 이것은 **PX4-Autopilot** 트리 외부 어디에든 둘 수 있습니다.
  - 이것은 **PX4-Autopilot**와 동일한 구조이어야 합니다. (예시 : **src** 디렉터리가 있어야 합니다.)
  - 이후 우리는 이 디렉터리를 `EXTERNAL_MODULES_LOCATION`이라고 하겠습니다.
- 기존 모듈을 (예: **examples/px4_simple_app**) 외부 디렉터리로 복사하거나 새 모듈을 바로 만드십시오.
- 모듈 이름을 바꾸거나(**CMakeLists.txt**내 `MODULE` 포함) 기존 PX4-Autopilot *cmake*의 빌드 설정에서 제거하십시오. 이것은 내부 모듈과의 참조 중복, 동시 자원 사용을 막기위함 입니다.
- 외부 디렉터리의 **CMakeLists.txt**에 다음 내용을 추가하십시오: 
      set(config_module_list_external
          modules/<new_module>
          PARENT_SCOPE
          )

- `EXTERNAL` 줄을 `modules/<new_module>/CMakeLists.txt`의 `px4_add_module()`에 추가하십시오. 예를 들면, 다음과 같습니다:
  
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
      

<a id="uorb_message_definitions"></a>

## 외부(Out-of-Tree) uORB 메시지 정의

uORB 메시지는 별도 메시지로 정의할 수도 있습니다. 이렇게 하려면 `$EXTERNAL_MODULES_LOCATION/msg` 폴더가 있어야합니다.

- 모든 새 메시지 정의를 `$EXTERNAL_MODULES_LOCATION/msg` 디렉터리에 넣으십시오. 이들 새 별도 메시지 정의 형식은 다른 [uORB 메시지 정의](../middleware/uorb.md#adding-a-new-topic)시에도 동일합니다.
- 다음 내용을 채워 넣은 `$EXTERNAL_MODULES_LOCATION/msg/CMakeLists.txt` 파일을 추가하십시오:
  
      set(config_msg_list_external
          <message1>.msg
          <message2>.msg
          <message3>.msg
          PARENT_SCOPE
          )
      
  
  `<message#>.msg` 부분은 uORB 메시지 생성시 처리하고 활용할 uORB 메시지 정의 파일의 이름입니다.

별도의 uORB 메시지는 일반 uORB 메시지와 동일한 위치에 생성합니다. uORB 토픽 헤더는 `<build_dir>/uORB/topics/`에 만들고, 메시지 소스 파일은 `<build_dir>/msg/topics_sources/`에 만듭니다.

[이곳](../middleware/uorb.md#adding-a-new-topic)에 설명한 바와 같이 새 uORB 메시지는 다른 uORB 메시지처럼 활용할 수 있습니다.

> **Warning** 외부 uORB 메시지 정의는 기존의 일반 uORB 메시지와 동일한 이름을 가질 수 없습니다.

<a id="building"></a>

## 외부 모듈과 uORB 메시지 구성

`make px4_sitl EXTERNAL_MODULES_LOCATION=<path>` 명령을 실행하십시오.

다른 빌드 대상을 활용할 수 있지만, 아직 빌드 디렉터리가 있으면 안됩니다. 이미 있다면 *cmake* 변수 값을 빌드 폴더에 설정할 수 있습니다.

차후 추가 빌드 과정에서는 `EXTERNAL_MODULES_LOCATION` 값을 지정할 필요가 없습니다.