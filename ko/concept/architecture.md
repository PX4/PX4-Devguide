# PX4의 구조적 개요

PX4는 2개의 주된 레이어로 구성됩니다. [flight stack](#flight-stack)은 비행제어를 추상화한 계층입니다. [미들웨어](#middleware)는 자율 로봇, 내/외부 통신, 하드웨어 통합을 지원하는 일반적인 레이어입니다.

모든 PX4 [기체](../airframes/README.md)는 하나의 codebase (보트, 로봇, 잠수함 등의 다른 로보틱스 시스템도 포함) 를 공유합니다. 완전한 시스템 디자인은 [reactive](http://www.reactivemanifesto.org)입니다. 무슨 뜻이나면

- 모든 기능들은 교환가능하고 재사용가능한 컴포넌트들로 나뉩니다.
- 통신은 비동기적인 메시지 전달에 의해 수행됩니다.
- 이 시스템은 바뀌는 변화에도 잘 견딥니다.

## 고수준 소프트웨어 아키텍쳐{#architecture}

아래의 다이어그램은 PX4를 구성하는 블럭들의 개요를 자세히 보여줍니다, 최상단의 다이어그램은 미들웨어 블럭들을 포함하고, 그 아래의 컴포넌트들은 flight stack을 나타냅니다.

![PX4 Architecture](../../assets/diagrams/PX4_Architecture.svg)

<!-- This diagram can be updated from 
[here](https://drive.google.com/file/d/0B1TDW9ajamYkaGx3R0xGb1NaeU0/view?usp=sharing) 
and opened with draw.io Diagrams. You might need to request access if you
don't have a px4.io Google account.
Caution: it can happen that after exporting some of the arrows are wrong. In
that case zoom into the graph until the arrows are correct, and then export
again. -->

The source code is split into self-contained modules/programs (shown in `monospace` in the diagram). 대게 하나의 블럭은 하나의 모듈과 완전히 일치합니다.

> **Tip** 실행중에, 쉘에서 `top` 명령어를 통해 실행중인 모듈을 검사할 수 있고, 각각의 모듈을 `<module_name> start/stop` 명령어를 통해서 시작/중지 시킬 수 있습니다. 하지만 `top` 명령어는 NuttX 쉘에서만 사용가능하고 다른 명령어들은 SITL 쉘 (pxh >) 에서도 사용할 수 있습니다. 각 모듈들에 대한 자세한 정보는[Modules & Commands Reference](../middleware/modules_main.md)를 참고하세요.

화살표는 모듈간의 *가장 중요한* 커넥션에 대한 정보의 흐름을 보여줍니다. 실제로는 표시된 것 보다 많은 커넥션들이 있고, 일부 데이터 (e.g. 파라미터) 들은 다수의 모듈들에 의해 접근됩니다.

모듈들은 [uORB](../middleware/uorb.md)라고 불리는 publish-subscribe 메시지 버스를 통해 각각 통신합니다. Publish-subscribe 스킴의 사용은 다음을 의미합니다.

- 이 시스템은 reactive 하다 - 비동기적이고 새로운 데이터가 이용가능해시면 즉시 업데이트 된다는 말입니다.
- 모든 연산과 통신들이 완전히 병렬화 되어있다.
- A system component can consume data from anywhere in a thread-safe fashion

> **Info** This architecture allows every single one of these blocks to be rapidly and easily replaced, even at runtime.

### Flight Stack {#flight-stack}

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

### Middleware {#middleware}

The [middleware](../middleware/README.md) consists primarily of device drivers for embedded sensors, communication with the external world (companion computer, GCS, etc.) and the uORB publish-subscribe message bus.

In addition, the middleware includes a [simulation layer](../simulation/README.md) that allows PX4 flight code to run on a desktop operating system and control a computer modeled vehicle in a simulated "world".

## Update Rates

Since the modules wait for message updates, typically the drivers define how fast a module updates. Most of the IMU drivers sample the data at 1kHz, integrate it and publish with 250Hz. Other parts of the system, such as the `navigator`, don't need such a high update rate, and thus run considerably slower.

The message update rates can be [inspected](../middleware/uorb.md) in real-time on the system by running `uorb top`.

## Runtime Environment

PX4 runs on various operating systems that provide a POSIX-API (such as Linux, macOS, NuttX or QuRT). It should also have some form of real-time scheduling (e.g. FIFO).

The inter-module communication (using [uORB](../middleware/uorb.md)) is based on shared memory. The whole PX4 middleware runs in a single address space, i.e. memory is shared between all modules.

> **Info** The system is designed such that with minimal effort it would be possible to run each module in separate address space (parts that would need to be changed include `uORB`, `parameter interface`, `dataman` and `perf`).

There are 2 different ways that a module can be executed:

- **Tasks**: The module runs in its own task with its own stack and process priority (this is the more common way). 
- **Work queues**: The module runs on a shared task, meaning that it does not own a stack. Multiple tasks run on the same stack with a single priority per work queue.
    
    A task is scheduled by specifying a fixed time in the future. The advantage is that it uses less RAM, but the task is not allowed to sleep or poll on a message.
    
    Work queues are used for periodic tasks, such as sensor drivers or the land detector.

> **Note** Tasks running on a work queue do not show up in `top` (only the work queues themselves can be seen - e.g. as `lpwork`).

### Background Tasks

`px4_task_spawn_cmd()` is used to launch new tasks (NuttX) or threads (POSIX - Linux/macOS) that run independently from the calling (parent) task:

```cpp
independent_task = px4_task_spawn_cmd(
    "commander",                    // Process name
    SCHED_DEFAULT,                  // Scheduling type (RR or FIFO)
    SCHED_PRIORITY_DEFAULT + 40,    // Scheduling priority
    3600,                           // Stack size of the new task or thread
    commander_thread_main,          // Task (or thread) main function
    (char * const *)&argv[0]        // Void pointer to pass to the new task
                                    // (here the commandline arguments).
    );
```

### OS-Specific Information

#### NuttX

[NuttX](http://nuttx.org/) is the primary RTOS for running PX4 on a flight-control board. It is open source (BSD license), light-weight, efficient and very stable.

Modules are executed as tasks: they have their own file descriptor lists, but they share a single address space. A task can still start one or more threads that share the file descriptor list.

Each task/thread has a fixed-size stack, and there is a periodic task which checks that all stacks have enough free space left (based on stack coloring).

#### Linux/macOS

On Linux or macOS, PX4 runs in a single process, and the modules run in their own threads (there is no distinction between tasks and threads as on NuttX).