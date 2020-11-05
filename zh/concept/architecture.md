# PX4 系统架构概述

PX4 由两个主要部分组成：一是 [飞行控制栈（flight stack）](#flight-stack) ，该部分主要包括状态估计和飞行控制系统；另一个是 [中间件](#middleware) ，该部分是一个通用的机器人应用层，可支持任意类型的自主机器人，主要负责机器人的内部/外部通讯和硬件整合。

所有的 PX4 支持的 [无人机机型](../airframes/README.md) （包括其他诸如无人船、无人车、无人水下航行器等平台）均共用同一个代码库。 整个系统采用了 [响应式（reactive）](http://www.reactivemanifesto.org) 设计，这意味着：

- 所有的功能都可以被分割成若干可替换、可重复使用的部件。
- 通过异步消息传递进行通信。
- 系统可以应对不同的工作负载。

<a id="architecture"></a>

## High-Level Software Architecture

The diagram below provides a detailed overview of the building blocks of PX4. The top part of the diagram contains middleware blocks, while the lower section shows the components of the flight stack.

![PX4 Architecture](../../assets/diagrams/PX4_Architecture.svg)

<!-- This diagram can be updated from 
[here](https://drive.google.com/file/d/0B1TDW9ajamYkaGx3R0xGb1NaeU0/view?usp=sharing) 
and opened with draw.io Diagrams. You might need to request access if you
don't have a px4.io Google account.
Caution: it can happen that after exporting some of the arrows are wrong. In
that case zoom into the graph until the arrows are correct, and then export
again. -->

The source code is split into self-contained modules/programs (shown in `monospace` in the diagram). Usually a building block corresponds to exactly one module.

> **Tip** 在运行期间，你可以在 shell 命令行界面执行 `top` 命令检查哪些模块已经被执行了， 每个模块都可以通过 `<module_name> start/stop` 命令单独进行启动/停止。 虽然 `top` 命令仅针对 NuttX shell，但其他命令是可以在 SITL shell (pxh>) 中使用的。 如果想获取每个模块的详细信息，请参阅 [Modules & Commands Reference](../middleware/modules_main.md) 。

The arrows show the information flow for the *most important* connections between the modules. In reality, there are many more connections than shown, and some data (e.g. for parameters) is accessed by most of the modules.

Modules communicate with each other through a publish-subscribe message bus named [uORB](../middleware/uorb.md). The use of the publish-subscribe scheme means that:

- The system is reactive — it is asynchronous and will update instantly when new data is available
- 系统所有的活动和通信都是完全并行的。
- 系统组件在任何地方都可以在保证线程安全的情况下使用数据。

> **Info** 这种体系架构使得系统中每一个组件都可以快速、轻松地实现替换，即便系统此时正在运行。

<a id="flight-stack"></a>

### Flight Stack

The flight stack is a collection of guidance, navigation and control algorithms for autonomous drones. It includes controllers for fixed wing, multirotor and VTOL airframes as well as estimators for attitude and position.

The following diagram shows an overview of the building blocks of the flight stack. It contains the full pipeline from sensors, RC input and autonomous flight control (Navigator), down to the motor or servo control (Actuators).

![PX4 High-Level Flight Stack](../../assets/diagrams/PX4_High-Level_Flight-Stack.svg) <!-- This diagram can be updated from 
[here](https://drive.google.com/a/px4.io/file/d/15J0eCL77fHbItA249epT3i2iOx4VwJGI/view?usp=sharing) 
and opened with draw.io Diagrams. You might need to request access if you
don't have a px4.io Google account.
Caution: it can happen that after exporting some of the arrows are wrong. In
that case zoom into the graph until the arrows are correct, and then export
again. -->

An **estimator** takes one or more sensor inputs, combines them, and computes a vehicle state (for example the attitude from IMU sensor data).

A **controller** is a component that takes a setpoint and a measurement or estimated state (process variable) as input. Its goal is to adjust the value of the process variable such that it matches the setpoint. The output is a correction to eventually reach that setpoint. For example the position controller takes position setpoints as inputs, the process variable is the currently estimated position, and the output is an attitude and thrust setpoint that move the vehicle towards the desired position.

A **mixer** takes force commands (e.g. turn right) and translates them into individual motor commands, while ensuring that some limits are not exceeded. This translation is specific for a vehicle type and depends on various factors, such as the motor arrangements with respect to the center of gravity, or the vehicle's rotational inertia.

<a id="middleware"></a>

### Middleware

The [middleware](../middleware/README.md) consists primarily of device drivers for embedded sensors, communication with the external world (companion computer, GCS, etc.) and the uORB publish-subscribe message bus.

In addition, the middleware includes a [simulation layer](../simulation/README.md) that allows PX4 flight code to run on a desktop operating system and control a computer modeled vehicle in a simulated "world".

## 更新速率

Since the modules wait for message updates, typically the drivers define how fast a module updates. Most of the IMU drivers sample the data at 1kHz, integrate it and publish with 250Hz. Other parts of the system, such as the `navigator`, don't need such a high update rate, and thus run considerably slower.

The message update rates can be [inspected](../middleware/uorb.md) in real-time on the system by running `uorb top`.

<a id="runtime-environment"></a>

## Runtime Environment

PX4 runs on various operating systems that provide a POSIX-API (such as Linux, macOS, NuttX or QuRT). It should also have some form of real-time scheduling (e.g. FIFO).

The inter-module communication (using [uORB](../middleware/uorb.md)) is based on shared memory. The whole PX4 middleware runs in a single address space, i.e. memory is shared between all modules.

> **Info** The system is designed such that with minimal effort it would be possible to run each module in separate address space (parts that would need to be changed include `uORB`, `parameter interface`, `dataman` and `perf`).

There are 2 different ways that a module can be executed:

- **Tasks**: The module runs in its own task with its own stack and process priority.
- **Work queue tasks**: The module runs on a shared work queue, sharing the same stack and work queue thread priority as other modules on the queue.
  
  - All the tasks must behave co-operatively as they cannot interrupt each other.
  - Multiple *work queue tasks* can run on a queue, and there can be multiple queues.
  - A *work queue task* is scheduled by specifying a fixed time in the future, or via uORB topic update callback.
  
  The advantage of running modules on a work queue is that it uses less RAM, and potentially results in fewer task switches. The disadvantages are that *work queue tasks* are not allowed to sleep or poll on a message, or do blocking IO (such as reading from a file). Long-running tasks (doing heavy computation) should potentially also run in a separate task or at least a separate work queue.

> **Note** Tasks running on a work queue do not show up in [`uorb top`](../middleware/modules_communication.md#uorb) (only the work queues themselves can be seen - e.g. as `wq:lp_default`). Use [`work_queue status`](../middleware/modules_system.md#workqueue) to display all active work queue items.

### 后台任务

`px4_task_spawn_cmd()` is used to launch new tasks (NuttX) or threads (POSIX - Linux/macOS) that run independently from the calling (parent) task:

```cpp
independent_task = px4_task_spawn_cmd(
    "commander",                    // 进程名称
    SCHED_DEFAULT,                  // 调度类型（RR 或 FIFO）
    SCHED_PRIORITY_DEFAULT + 40,    // 调度优先级
    3600,                           // 新任务或线程的堆栈大小
    commander_thread_main,          // 任务（或线程的主函数）
    (char * const *)&argv[0]        // Void 指针传递到新任务
                                    // （这里是命令行参数）
    );
```

### 操作系统相关的信息

#### NuttX

[NuttX](http://nuttx.org/) is the primary RTOS for running PX4 on a flight-control board. It is open source (BSD license), light-weight, efficient and very stable.

Modules are executed as tasks: they have their own file descriptor lists, but they share a single address space. A task can still start one or more threads that share the file descriptor list.

Each task/thread has a fixed-size stack, and there is a periodic task which checks that all stacks have enough free space left (based on stack coloring).

#### Linux/MacOS

On Linux or macOS, PX4 runs in a single process, and the modules run in their own threads (there is no distinction between tasks and threads as on NuttX).