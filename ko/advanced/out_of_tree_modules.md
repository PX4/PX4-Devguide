# 외부 모듈(트리 외부-Out-of-Tree)

외부 모듈에서는 PX4 펌웨어에 추가하려는 상용 모듈을 관리하고 그룹화하는 편의 매커니즘을 개발자에게 제공합니다. 외부 모듈은 내부 모듈과 마찬가지로 uORB를 통해 내부 모듈과 상호 메시지 교환을 할 수 있도록 활용할 수 있습니다.

이 주제에서는 외부("out of tree") 모듈을 PX4 빌드에 추가하는 방법을 설명합니다.

> **Tip** 가능하다면 PX4 프로젝트에 바뀐 내용을 기여해주시기 바랍니다!

## 사용법

외부 모듈을 만들려면:

- 외부 모듈을 모아둘 *외부 디렉터리*를 만드십시오: 
  - This can be located anywhere outside of the **PX4-Autopilot** tree.
  - It must have the same structure as **PX4-Autopilot** (i.e. it must contain a directory called **src**).
  - 이후 우리는 이 디렉터리를 `EXTERNAL_MODULES_LOCATION`이라고 하겠습니다.
- 기존 모듈을 (예: **examples/px4_simple_app**) 외부 디렉터리로 복사하거나 새 모듈을 바로 만드십시오.
- Rename the module (including `MODULE` in **CMakeLists.txt**) or remove it from the existing PX4-Autopilot *cmake* build config. This is to avoid conflicts with internal modules.
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

## Out-of-Tree uORB Message Definitions

uORB messages can also be defined out-of-tree. For this, the `$EXTERNAL_MODULES_LOCATION/msg` folder must exist.

- 모든 새 메시지 정의를 `$EXTERNAL_MODULES_LOCATION/msg` 디렉터리에 넣으십시오. 이들 새 별도 메시지 정의 형식은 다른 [uORB 메시지 정의](../middleware/uorb.md#adding-a-new-topic)시에도 동일합니다.
- 다음 내용을 채워 넣은 `$EXTERNAL_MODULES_LOCATION/msg/CMakeLists.txt` 파일을 추가하십시오:
  
      set(config_msg_list_external
          <message1>.msg
          <message2>.msg
          <message3>.msg
          PARENT_SCOPE
          )
      
  
  `<message#>.msg` 부분은 uORB 메시지 생성시 처리하고 활용할 uORB 메시지 정의 파일의 이름입니다.

The out-of-tree uORB messages will be generated in the same locations as the normal uORB messages. The uORB topic headers are generated in `<build_dir>/uORB/topics/`, and the message source files are generated in `<build_dir>/msg/topics_sources/`.

The new uORB messages can be used like any other uORB message as described [here](../middleware/uorb.md#adding-a-new-topic).

> **Warning** 외부 uORB 메시지 정의는 기존의 일반 uORB 메시지와 동일한 이름을 가질 수 없습니다.

<a id="building"></a>

## Building External Modules and uORB Messages

Execute `make px4_sitl EXTERNAL_MODULES_LOCATION=<path>`.

Any other build target can be used, but the build directory must not yet exist. If it already exists, you can also just set the *cmake* variable in the build folder.

For subsequent incremental builds `EXTERNAL_MODULES_LOCATION` does not need to be specified.