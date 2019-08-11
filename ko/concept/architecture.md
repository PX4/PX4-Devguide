# PX4의 구조적 개요

PX4는 2개의 주된 레이어로 구성됩니다. [flight stack](#flight-stack)은 비행제어를 추상화한 계층입니다. [미들웨어](#middleware)는 자율 로봇, 내/외부 통신, 하드웨어 통합을 지원하는 일반적인 레이어입니다.

모든 PX4 [기체](../airframes/README.md)는 하나의 codebase (보트, 로봇, 잠수함 등의 다른 로보틱스 시스템도 포함) 를 공유합니다. 완전한 시스템 디자인은 [reactive](http://www.reactivemanifesto.org)입니다. 무슨 뜻이나면

- 모든 기능들은 교환가능하고 재사용가능한 컴포넌트들로 나뉩니다.
- 통신은 비동기적인 메시지 전달에 의해 수행됩니다.
- 이 시스템은 바뀌는 변화에도 잘 견딥니다.

## High-Level Software Architecture{#architecture}

아래의 다이어그램은 PX4를 구성하는 블럭들의 개요를 자세히 보여줍니다, 최상단의 다이어그램은 미들웨어 블럭들을 포함하고, 그 아래의 컴포넌트들은 flight stack을 나타냅니다.

![PX4 Architecture](../../assets/diagrams/PX4_Architecture.svg)

<!-- This diagram can be updated from 
[here](https://drive.google.com/file/d/0B1TDW9ajamYkaGx3R0xGb1NaeU0/view?usp=sharing) 
and opened with draw.io Diagrams. You might need to request access if you
don't have a px4.io Google account.
Caution: it can happen that after exporting some of the arrows are wrong. In
that case zoom into the graph until the arrows are correct, and then export
again. -->

소스코드는 독립적인 모듈/프로그램(다이어그램에서 블럭들로 표시) 으로 나뉩니다. 대게 하나의 블럭은 하나의 모듈과 완전히 일치합니다.

> **Tip** 실행중에, 쉘에서 `top` 명령어를 통해 실행중인 모듈을 검사할 수 있고, 각각의 모듈을 `<module_name> start/stop` 명령어를 통해서 시작/중지 시킬 수 있습니다. 하지만 `top` 명령어는 NuttX 쉘에서만 사용가능하고 다른 명령어들은 SITL 쉘 (pxh >) 에서도 사용할 수 있습니다. 각 모듈들에 대한 자세한 정보는[Modules & Commands Reference](../middleware/modules_main.md)를 참고하세요.

화살표는 모듈간의 *가장 중요한* 커넥션에 대한 정보의 흐름을 보여줍니다. 실제로는 표시된 것 보다 많은 커넥션들이 있고, 일부 데이터 (e.g. 파라미터) 들은 다수의 모듈들에 의해 접근됩니다.

모듈들은 [uORB](../middleware/uorb.md)라고 불리는 publish-subscribe 메시지 버스를 통해 각각 통신합니다. Publish-subscribe 스킴의 사용은 다음을 의미합니다.

- 이 시스템은 reactive 하다 - 비동기적이고 새로운 데이터가 이용가능해시면 즉시 업데이트 된다는 말입니다.
- 모든 연산과 통신들이 완전히 병렬화 되어있다.
- 시스템의 컴포넌트는 thread-safe 방식으로 어디에서든 data를 사용할 수 있다.

> **Info** 이 아키텍처는 이러한 블록 중 하나를 런타임시에도 빠르고 쉽게 교체 할 수 있습니다.

### Flight Stack {#flight-stack}

Flight Stack은 자율적인 드론을 위한 유도, 네비게이션, 컨트롤 알고리즘의 콜렉션입니다. 고정익, 멀티로터, VTOL 기체, 게다가 균형이나 위치를 위한 컨트롤러를 포함합니다.

아래의 그림은 flight stack을 구성하는 블록들의 개요를 보여줍니다. 센서, RC 입력 및 자율 비행 제어 (네비게이터) 에서 모터 또는 서보 제어 (액추에이터) 까지의 전체 파이프라인을 포함합니다.

![PX4 High-Level Flight Stack](../../assets/diagrams/PX4_High-Level_Flight-Stack.svg) <!-- This diagram can be updated from 
[here](https://drive.google.com/a/px4.io/file/d/15J0eCL77fHbItA249epT3i2iOx4VwJGI/view?usp=sharing) 
and opened with draw.io Diagrams. You might need to request access if you
don't have a px4.io Google account.
Caution: it can happen that after exporting some of the arrows are wrong. In
that case zoom into the graph until the arrows are correct, and then export
again. -->

**estimator**는 하나 이상의 센서 입력을 받아들이고 결합하여 기체를 상태를 계산합니다 (예를 들어 IMU 센서의 데이터로 부터 기체의 자세를 계산하는 것).

**컨트롤러**는 설정값, 측정장 또는 추정값 (측정된 값을 처리한 값) 을 입력으로 받아들이는 컴포넌트 입니다. 이 장치의 목표는 Process variable을 조정해 설정값과 일치해지게 만다는 것입니다. 출력은 설정값에 도달하기 위해 보정된 값입니다. 예를 들어, 포지션 컨트롤러가 설정값을 입력으로 받습니다. process variable은 현재의 위치를 추정한 값입니다. 그리고 출력은 기체를 목표한 위치로 움직이기 위한 자세와 추력 설정값입니다.

**mixer**는 명령 (예. 우회전) 을 받아들이고 해석하여 개별 모터 명령으로 변환하지만 어떤 제한을 넘지 않도록 합니다. 이 변환은 기체의 타입, 무게 중심을 기준으로한 모터의 배열, 기체의 회전 관성과 같은 요소들에 의존적입니다.

### Middleware {#middleware}

[middleware](../middleware/README.md) 는 주로 임베디드 센서, 외부와의 통신 (companion computer, GCS, 등), uORB publish-subscribe 메시지 버스를 위한 드바이스 드라이버들로 구성됩니다.

게다가, middleware는 [simulation layer](../simulation/README.md)를 포함합니다. simulation layer는 데스크탑 OS에서 PX4 코드의 실행과 가상으로 구현된 기체의 제어를 지원합니다.

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