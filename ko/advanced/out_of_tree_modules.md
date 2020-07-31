# 외부 모듈(트리 외부-Out-of-Tree)

외부 모듈에서는 PX4 펌웨어에 추가하려는 상용 모듈을 관리하고 그룹화하는 편의 매커니즘을 개발자에게 제공합니다. 외부 모듈은 내부 모듈과 마찬가지로 uORB를 통해 내부 모듈과 상호 메시지 교환을 할 수 있도록 활용할 수 있습니다.

이 주제에서는 외부("out of tree")모듈을 PX4 빌드에 추가하는 방법을 설명합니다.

> **Tip** 가능하다면 PX4에 바뀐 내용을 기여해주시기 바랍니다!

## 사용법

외부 모듈을 만들려면:

- 외부 모듈을 모아둘 *외부 디렉터리*를 만드십시오: 
  - **펌웨어** 트리 외부 어디에든 둘 수 있습니다.
  - **펌웨어**와 동일한 구조를 가져야합니다(예시: **src** 디렉터리가 있어야합니다).
  - 이후 우리는 이 디렉터리를 `EXTERNAL_MODULES_LOCATION`이라고 하겠습니다.
- 기존 모듈을 (예: **examples/px4_simple_app**) 외부 디렉터리로 복사하거나 새 모듈을 바로 만드십시오.
- Rename the module (including `MODULE` in **CMakeLists.txt**) or remove it from the existing Firmware *cmake* build config. This is to avoid conflicts with internal modules.
- Add a file **CMakeLists.txt** in the external directory with content: 
      set(config_module_list_external
          modules/<new_module>
          PARENT_SCOPE
          )

- Add a line `EXTERNAL` to the `modules/<new_module>/CMakeLists.txt` within `px4_add_module()`, for example like this:
  
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
      

## Out-of-Tree uORB Message Definitions {#uorb_message_definitions}

uORB messages can also be defined out-of-tree. For this, the `$EXTERNAL_MODULES_LOCATION/msg` folder must exist.

- Place all new message definitions within the `$EXTERNAL_MODULES_LOCATION/msg` directory. The format of these new out-of-tree message definitions are the same as for any other [uORB message definition](../middleware/uorb.md#adding-a-new-topic).
- Add a file `$EXTERNAL_MODULES_LOCATION/msg/CMakeLists.txt` with content:
  
      set(config_msg_list_external
          <message1>.msg
          <message2>.msg
          <message3>.msg
          PARENT_SCOPE
          )
      
  
  where `<message#>.msg` is the name of the uORB message definition file to be processed and used for uORB message generation.

The out-of-tree uORB messages will be generated in the same locations as the normal uORB messages. The uORB topic headers are generated in `<build_dir>/uORB/topics/`, and the message source files are generated in `<build_dir>/msg/topics_sources/`.

The new uORB messages can be used like any other uORB message as described [here](../middleware/uorb.md#adding-a-new-topic).

> **Warning** The out-of-tree uORB message definitions cannot have the same name as any of the normal uORB messages.

## Building External Modules and uORB Messages {#building}

Execute `make px4_sitl EXTERNAL_MODULES_LOCATION=<path>`.

Any other build target can be used, but the build directory must not yet exist. If it already exists, you can also just set the *cmake* variable in the build folder.

For subsequent incremental builds `EXTERNAL_MODULES_LOCATION` does not need to be specified.