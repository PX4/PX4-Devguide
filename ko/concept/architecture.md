# PX4 아키텍처 개요

PX4는 주요 계층 2가지로 구성했습니다. [플라이트 스택](#flight-stack)은 추정, 비행 제어 체계이며, [미들웨어](#middleware)는 내외부 통신과 하드웨어간 결합 기능을 지원하는 여러가지 자동 처리 로봇을 지원할 수 있는 범용 로보틱스 계층입니다.

모든 PX4 [기체](../airframes/README.md)는 하나의 코드 기반(보트, 로봇, 잠수함 등의 다른 로보틱스 시스템도 포함)을 공유합니다. 완전한 시스템 디자인은 [반응형](http://www.reactivemanifesto.org)입니다. 무슨 뜻이나면:

- 모든 기능은 대체, 재사용 가능 요소로 나눕니다.
- 비동기 메시지 전달로 통신을 수행합니다.
- 다양한 부하 작업을 처리할 수 있습니다.

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

> **Tip** 실행 중에, 셸에서 `top` 명령으로 어떤 모듈을 실행하는지 볼 수 있으며, 어떤 모듈을 `<module_name> start/stop` 명령으로 제각각 시작하고 멈출 수 있는지 확인할 수 있습니다. `top` 명령어는 NuttX 쉘에서만 쓸 수 있지만, 다른 명령어들은 SITL 쉘(pxh>)에서도 사용할 수 있습니다. 이 모듈에 대한 더 많은 내용은 [모듈 & 명령 참고](../middleware/modules_main.md)를 참고하십시오. 

The arrows show the information flow for the *most important* connections between the modules. In reality, there are many more connections than shown, and some data (e.g. for parameters) is accessed by most of the modules.

Modules communicate with each other through a publish-subscribe message bus named [uORB](../middleware/uorb.md). The use of the publish-subscribe scheme means that:

- 반응형 시스템 — 비동기 방식으로 동작하며 새 데이터를 넣으면 바로 업데이트합니다
- 모든 연산, 통신 동작에 대해 완전한 병렬 처리를 수행합니다
- 시스템 구성요소는 스레드 처리에 안전한 방식으로 어디에서든 데이터를 활용할 수 있습니다.

> **Info** 이 아키텍처는 실행 시간중에도 모든 단일 블록을 빠르고 쉽게 대체하도록 합니다.

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

## 업데이트 속도

Since the modules wait for message updates, typically the drivers define how fast a module updates. Most of the IMU drivers sample the data at 1kHz, integrate it and publish with 250Hz. Other parts of the system, such as the `navigator`, don't need such a high update rate, and thus run considerably slower.

The message update rates can be [inspected](../middleware/uorb.md) in real-time on the system by running `uorb top`.

<a id="runtime-environment"></a>

## Runtime Environment

PX4 runs on various operating systems that provide a POSIX-API (such as Linux, macOS, NuttX or QuRT). It should also have some form of real-time scheduling (e.g. FIFO).

The inter-module communication (using [uORB](../middleware/uorb.md)) is based on shared memory. The whole PX4 middleware runs in a single address space, i.e. memory is shared between all modules.

> **Info** 시스템은 개별 주소 공간에서 각 모듈을 실행하는데 최소한의 비용이 들어가도록 설계했습니다(`uORB`, `매개변수 인터페이스`, `dataman`, `perf` 같은 부분을 조금 바꿔야 합니다).

There are 2 different ways that a module can be executed:

- **작업**: 모듈은 자체 스택을 확보하고 프로세스 우선순위를 부여받아 자체 작업내에서 실행합니다.
- **작업 큐(work queue)의 작업**: 모듈은 동일한 스택과 큐의 다른 모듈처럼 작업 큐 스레드 우선순위를 부여받은 공유 작업 큐에서 실행합니다.
  
  - 모든 작업은 서로의 동작을 중단하지 않고 사이좋게 동작해야합니다.
  - 다중 *작업 큐 작업*은 큐에서 실행할 수 있으며, 다중 큐가 될 수 있습니다.
  - *작업 큐의 작업*은 향후 지정 시간에 동작하도록 계획하거나 uORB 토픽 업데이트 콜백으로 처리합니다.
  
  실행 큐에서 모듈을 실행하는 장점은 RAM 소모량이 적고, 잠재적으로 작업 전환이 빈번하지 않는다는 점입니다. 단점은 *작업 큐 작업*을 대기 상태로 두거나 메시지를 폴링하거나 입출력을 멈출(파일 읽기 등) 수 없습니다. 장시간 실행 작업(막대한 양의 계산처리 수행)의 경우 잠재적으로 개별 작업으로 분리하여 실행하거나 최소한 작업 큐를 분할해야 합니다.

> **Note** 작업 큐에서 실행하는 작업은 [`uorb top`](../middleware/modules_communication.md#uorb)에 나타나지 않습니다(작업 큐 자체는 `wq:lp_default`처럼 나타날 수는 있습니다). 모든 활성 작업 큐 항목을 보려면 [`work_queue status`](../middleware/modules_system.md#workqueue) 명령을 활용하십시오.

### 백그라운드 작업

`px4_task_spawn_cmd()` is used to launch new tasks (NuttX) or threads (POSIX - Linux/macOS) that run independently from the calling (parent) task:

```cpp
ndependent_task = px4_task_spawn_cmd(
    "commander",                    // Process name
    SCHED_DEFAULT,                  // Scheduling type (RR or FIFO)
    SCHED_PRIORITY_DEFAULT + 40,    // Scheduling priority
    3600,                           // Stack size of the new task or thread
    commander_thread_main,          // Task (or thread) main function
    (char * const *)&argv[0]        // Void pointer to pass to the new task
                                    // (here the commandline arguments).
    );
```

### 운영체제별 정보

#### NuttX

[NuttX](http://nuttx.org/) is the primary RTOS for running PX4 on a flight-control board. It is open source (BSD license), light-weight, efficient and very stable.

Modules are executed as tasks: they have their own file descriptor lists, but they share a single address space. A task can still start one or more threads that share the file descriptor list.

Each task/thread has a fixed-size stack, and there is a periodic task which checks that all stacks have enough free space left (based on stack coloring).

#### Linux/macOS

On Linux or macOS, PX4 runs in a single process, and the modules run in their own threads (there is no distinction between tasks and threads as on NuttX).