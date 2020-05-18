# 适用于开发完整应用的模版

An application can be written to run as either a *task* (a module with its own stack and process priority) or as a *work queue task* (a module that runs on a work queue thread, sharing the stack and thread priorit with other tasks on the work queue). In most cases a work queue task can be used, as this minimizes resource usage.

> **Note** [Architectural Overview > Runtime Environment](../concept/architecture.md#runtime-environment) provides more information about tasks and work queue tasks.

<span></span>

> **Note** All the things learned in the [First Application Tutorial](../apps/hello_sky.md) are relevant for writing a full application.

## Work Queue Task

The PX4 Firmware contains a template for writing a new application (module) that runs as a *work queue task*: [src/examples/work_item](https://github.com/PX4/Firmware/tree/master/src/examples/work_item).

A work queue task application is just the same as an ordinary (task) application, except that it needs to specify that it is a work queue task, and schedule itself to run during initialisation.

The example shows how. In summary:

1. Specify the dependency on the work queue library in the cmake definition file ([CMakeLists.txt](https://github.com/PX4/Firmware/blob/master/src/examples/work_item/CMakeLists.txt)): 
        ...
        DEPENDS
          px4_work_queue

2. In addition to `ModuleBase`, the task should also derive from `ScheduledWorkItem` (included from [ScheduledWorkItem.hpp](https://github.com/PX4/Firmware/blob/master/platforms/common/include/px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp))
3. Specify the queue to add the task to in the constructor initialisation. The [work_item](https://github.com/PX4/Firmware/blob/master/src/examples/work_item/WorkItemExample.cpp#L42) example adds itself to the `wq_configurations::test1` work queue as shown below:
    
    ```cpp
    WorkItemExample::WorkItemExample() :
       ModuleParams(nullptr),
       ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
    {
    }
    ```
    
    > **Note** The available work queues (`wq_configurations`) are listed in [WorkQueueManager.hpp](https://github.com/PX4/Firmware/blob/master/platforms/common/include/px4_platform_common/px4_work_queue/WorkQueueManager.hpp#L49).

4. Implement the `ScheduledWorkItem::Run()` method to perform "work".

5. Implement the `task_spawn` method, specifying that the task is a work queue (using the `task_id_is_work_queue` id.
6. Schedule the work queue task using one of the scheduling methods (in the example we use `ScheduleOnInterval` from within the `init` method).

## Tasks

The PX4 Firmware contains a template for writing a new application (module) that runs as a task on its own stack: [src/templates/module](https://github.com/PX4/Firmware/tree/master/src/templates/module).

The template demonstrates the following additional features/aspects that are required or are useful for a full application:

- 访问参数并对参数更新做出反应。
- 订阅、等待 topic 更新。
- 通过 `start`/`stop`/`status` 控制后台运行的任务。 `module start [<arguments>]` 命令可以直接加入 [启动脚本](../concept/system_startup.md) 中。
- 命令行参数解析。
- 文档记录：`PRINT_MODULE_*` 方法有两个用处（该 API 在 [源代码](https://github.com/PX4/Firmware/blob/v1.8.0/src/platforms/px4_module.h#L381) 中有详细记录）： 
    - 它们可用于在控制台键入 `module help ` 指令后输出命令行指令的用法。
    - 可通过脚本提取该部分内容以自动生成 [Modules & Commands Reference](../middleware/modules_main.md) 页面。