# 전체 프로그램용 모듈 서식

프로그램은 *작업* 단위(모듈 자체에 스택을 보유하고 있고, 프로세스 우선순위가 존재)로 또는 *작업 큐의 작업*(작업 큐 스레드에서 동작하는 모듈이며, 스택을 공유하고 작업 큐 대기시 다른 작업들 간의 스레드 우선순위를 가짐)으로 동작하도록 작성할 수 있습니다. 대부분의 경우 작업 큐에서 실행하는 방법이 자원 소모를 최소화하므로 이 방법을 활용할 수 있습니다.

> **Note** [구조 개요 > 실행 시간 환경](../concept/architecture.md#runtime-environment) 항목에서 작업과 작업 큐의 작업에 대해 더 알아볼 수 있습니다.

<span></span>

> **Note** [첫 프로그램 자습서](../apps/hello_sky.md)에서 배운 모든 내용은 전체 프로그램 작성에도 관련있습니다.

## 작업 큐의 작업

PX4-Autopilot contains a template for writing a new application (module) that runs as a *work queue task*: [src/examples/work_item](https://github.com/PX4/PX4-Autopilot/tree/master/src/examples/work_item).

작업 큐 작업 프로그램은 그냥 일반 (작업) 프로그램과 동일하나, 작업 큐의 작업이란 점을 지정하고 초기화 단계에서 자체적으로 스케쥴링하는 점만 다릅니다.

예제를 통해 어떻게 하는지 확인합니다. 요약하자면:

1. Specify the dependency on the work queue library in the cmake definition file ([CMakeLists.txt](https://github.com/PX4/PX4-Autopilot/blob/master/src/examples/work_item/CMakeLists.txt)): 
        ...
        DEPENDS
          px4_work_queue

2. In addition to `ModuleBase`, the task should also derive from `ScheduledWorkItem` (included from [ScheduledWorkItem.hpp](https://github.com/PX4/PX4-Autopilot/blob/master/platforms/common/include/px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp))
3. 생성자 초기화시, 작업을 추가할 큐를 지정하십시오. The [work_item](https://github.com/PX4/PX4-Autopilot/blob/master/src/examples/work_item/WorkItemExample.cpp#L42) example adds itself to the `wq_configurations::test1` work queue as shown below:
    
    ```cpp
    WorkItemExample::WorkItemExample() :
       ModuleParams(nullptr),
       ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
    {
    }
    ```
    
    > **Note** The available work queues (`wq_configurations`) are listed in [WorkQueueManager.hpp](https://github.com/PX4/PX4-Autopilot/blob/master/platforms/common/include/px4_platform_common/px4_work_queue/WorkQueueManager.hpp#L49).

4. "작업"을 수행할 `ScheduledWorkItem::Run()` 메서드를 구현하십시오.

5. 작업을 작업 큐(`task_id_is_work_queue` id 사용)에 지정하는 `task_spawn` 메서드를 구현하십시오.
6. 스케줄링 메서드중 하나를 사용하여 work queue task를 스케쥴링함 (예제에서는 `init` 메서드내의 `ScheduleOnInterval`를 사용함).

## 작업

PX4/PX4-Autopilot contains a template for writing a new application (module) that runs as a task on its own stack: [src/templates/template_module](https://github.com/PX4/PX4-Autopilot/tree/master/src/templates/template_module).

서식에서는 완전한 프로그램 작성에 필요하거나 쓸만한 다음 추가 기능이나 양상을 보여줍니다:

- 배개변수 접근, 매개변수 업데이트에 대응
- uORB 정기 수신 및 토픽 업데이트 대기
- `start`/`stop`/`status`로 백그라운드에 실행하는 작업 제어 `module start [<arguments>]` 명령은 [시작 스크립트에](../concept/system_startup.md) 직접 추가할 수 있습니다.
- 명령행 인자 파싱.
- Documentation: the `PRINT_MODULE_*` methods serve two purposes (the API is documented [in the source code](https://github.com/PX4/PX4-Autopilot/blob/v1.8.0/src/platforms/px4_module.h#L381)): 
    - 콘솔상에 `module help` 입력시 해당 명령 사용법 출력을 위해 사용됨.
    - 스크립트를 통해 자동으로 추출하여 [Modules & Commands Reference](../middleware/modules_main.md) 페이지를 생성.