# Module Template for Full Applications

어플리케이션은 *task* (자신의 스택과 수행 우선순위를 지니는 모듈)나 *work queue task* (작업 큐 쓰레드상에서 실행되는 모듈로, 작업큐 내의 다른 여러 태스크와 스택과 우선순위를 공유함)로 작성할 수 있습니다. 리소스 사용을 최소화하기 때문에 대부분의 경우 work queue task를 사용할 수 있습니다.

> **Note** [Architectural Overview > Runtime Environment](../concept/architecture.md#runtime-environment) 항목에서 task와 work queue task에 대한 자세한 정보를 제공.

<span></span>

> **Note** [첫 번째 어플리케이션 튜토리얼](../apps/hello_sky.md)에서 배운 내용은 Full 어플리케이션 작성에도 연관됨.

## Work Queue Task

PX4 Firmware은 *work queue task*로 동작하는 신규 어플리케이션(모듈) 작성용 템플릿을 포함: [src/examples/work_item](https://github.com/PX4/Firmware/tree/master/src/examples/work_item).

Work queue task 어플리케이션은 보통의 (task) 어플리케이션과 동일하나, 이 어플리케이션이 work queue task임을 명시하고 초기화 구간에서 자신을 스케쥴할 필요가 있습니다.

예제를 통해 어떻게 하는지 확인합니다. 요약하면:

1. Cmake 정의 파일([CMakeLists.txt](https://github.com/PX4/Firmware/blob/master/src/examples/work_item/CMakeLists.txt))에 work queue 라이브러리 의존성을 명시함: 
        ...
        DEPENDS
          px4_work_queue

2. 태스크는 `ModuleBase`에 추가로 ([ScheduledWorkItem.hpp](https://github.com/PX4/Firmware/blob/master/platforms/common/include/px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp)에 포함된)`ScheduledWorkItem`에서도 파생되어야 함
3. 생성자 초기화시에 태스크를 추가할 큐를 명시함. [work_item](https://github.com/PX4/Firmware/blob/master/src/examples/work_item/WorkItemExample.cpp#L42) 예제에서는 자신을 `wq_configurations::test1` work queue에 아래와 같이 추가함:
    
    ```cpp
    WorkItemExample::WorkItemExample() :
       ModuleParams(nullptr),
       ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
    {
    }
    ```
    
    > **Note** 사용가능한 work queues(`wq_configurations`)는 [WorkQueueManager.hpp](https://github.com/PX4/Firmware/blob/master/platforms/common/include/px4_platform_common/px4_work_queue/WorkQueueManager.hpp#L49)에 나열됨.

4. "작업(work)"을 수행할 `ScheduledWorkItem::Run()` 메서드를 구현.

5. `task_spawn` 메서드를 구현. 해당 task가 work queue임을 명시 (`task_id_is_work_queue` id 사용).
6. 스케줄링 메서드중 하나를 사용하여 work queue task를 스케쥴링함 (예제에서는 `init` 메서드내의 `ScheduleOnInterval`를 사용함).

## Tasks

PX4 Firmware은 task로 동작하는 신규 어플리케이션(모듈) 작성용 템플릿을 포함: [src/templates/module](https://github.com/PX4/Firmware/tree/master/src/templates/module).

템플릿에서는 Full 어플리케이션에서 요구되거나 혹은 필요한 다음의 추가적인 기능/측면을 보여줌:

- 파라미터 접근 및 파라미터 업데이트에 대한 대응 동작
- uORB 구독(subscription) 및 topic 업데이트 대기.
- `start`/`stop`/`status`를 통해 백그라운드에서 실행되는 task 제어. `module start [<arguments>]` 명령은 [startup script](../concept/system_startup.md) 직접적으로 추가 가능.
- 명령행 인자 파싱.
- Documentation: the `PRINT_MODULE_*` methods serve two purposes (the API is documented [in the source code](https://github.com/PX4/Firmware/blob/v1.8.0/src/platforms/px4_module.h#L381)): 
    - They are used to print the command-line usage when entering `module help` on the console.
    - They are automatically extracted via script to generate the [Modules & Commands Reference](../middleware/modules_main.md) page.