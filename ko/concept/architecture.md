# PX4 아키텍처 개요

PX4는 2개의 주된 레이어로 구성됩니다. [flight stack](#flight-stack)은 비행제어를 추상화한 계층입니다. [middle](#middleware)는 자율 로봇, 내/외부 통신, 하드웨어 통합을 지원하는 일반적인 레이어입니다.

모든 PX4 [기체](../airframes/README.md)는 하나의 codebase (보트, 로봇, 잠수함 등의 다른 로보틱스 시스템도 포함) 를 공유합니다. 완전한 시스템 디자인은 [reactive](http://www.reactivemanifesto.org)입니다. 무슨 뜻이나면

- 모든 기능들은 교환가능하고 재사용가능한 컴포넌트들로 나뉩니다.
- 통신은 비동기적인 메시지 전달에 의해 수행됩니다.
- 이 시스템은 다양한 작업을 쉽게 다룰 수 있습니다.

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

Flight Stack은 자율적인 드론을 위한 유도, 네비게이션, 컨트롤 알고리즘의 집합입니다. 고정익, 멀티로터, VTOL 기체, 게다가 균형이나 위치를 위한 컨트롤러를 포함합니다.

아래의 그림은 flight stack을 구성하는 블록들의 개요를 보여줍니다. 센서, RC 입력 및 자율 비행 제어 (네비게이터) 에서 모터 또는 서보 제어 (액추에이터) 까지의 전체 파이프라인을 포함합니다.

![PX4 High-Level Flight Stack](../../assets/diagrams/PX4_High-Level_Flight-Stack.svg) <!-- This diagram can be updated from 
[here](https://drive.google.com/a/px4.io/file/d/15J0eCL77fHbItA249epT3i2iOx4VwJGI/view?usp=sharing) 
and opened with draw.io Diagrams. You might need to request access if you
don't have a px4.io Google account.
Caution: it can happen that after exporting some of the arrows are wrong. In
that case zoom into the graph until the arrows are correct, and then export
again. -->

**estimator**는 하나 이상의 센서 입력을 받아들이고 결합하여 기체를 상태를 계산합니다 (예를 들어 IMU 센서의 데이터로 부터 기체의 자세를 계산하는 것).

**컨트롤러**는 설정값, 측정장 또는 추정값 (측정된 값을 처리한 값) 을 입력으로 받아들이는 컴포넌트 입니다. 이 장치의 목표는 Process variable을 조정해 설정값과 일치하게 만다는 것입니다. 출력은 설정값에 도달하기 위해 보정된 값입니다. 예를 들어, 포지션 컨트롤러가 설정값을 입력으로 받습니다. process variable은 현제의 위치를 추정한 값입니다. 그리고 출력은 기체를 목표한 위치로 움직이기 위한 자세와 추력 설정값입니다.

**mixer**는 명령 (예. 우회전) 을 받아들이고 해석하여 개별 모터 명령으로 변환하지만 어떤 제한을 넘지 않도록 합니다. 이 변환은 기체의 타입, 무게 중심을 기준으로한 모터의 배열, 기체의 회전 관성과 같은 요소들에 의존적입니다.

### Middleware {#middleware}

[middleware](../middleware/README.md) 는 주로 임베디드 센서, 외부와의 통신 (companion computer, GCS, 등), uORB publish-subscribe 메시지 버스를 위한 드바이스 드라이버들로 구성됩니다.

게다가, middleware는 [simulation layer](../simulation/README.md)를 포함합니다. simulation layer는 데스크탑 OS에서 PX4 코드의 실행과 가상으로 구현된 기체의 제어를 지원합니다.

## Update Rates

모듈들은 메시지를 업데이트를 기다리므로, 보통 드라이버는 모듈 업데이트 속도를 정의합니다. 대부분의 IMU 드라이버는 1kHZ로 데이터를 샘플링하고 합하여, 250Hz로 공표합니다. `navigator`와 같은 시스템의 다른 부분들은 빨리 업데이트할 필요가 없어서 상대적으로 느리게 수행합니다.

메시지의 업데이트 속도는 시스템의 `uORB top`에 의해 실시간으로 [검사](../middleware/uorb.md) 됩니다.

## Runtime Environment

PX4는 POSIX-API를 지원하는 다양한 OS에서 실행됩니다 ( 리눅스, macOS, NuttX, QuRT). OS들은 반드시 실시간 스케쥴링 (예, FIFO) 형태를 갖고 있어야 합니다.

[uORB](../middleware/uorb.md)을 이용한 모듈간 통신은 공유 메모리를 기초로 합니다. PX4 middleware 전체는 하나의 주소공간에서 실행됩니다. 메모리가 모든 모듈이게 공유되는 것 입니다.

> **Info** 이 시스템은 각 모듈이 독립된 공간에서 실행시키는 것이 가능도록 최소한의 자원을 소모하도록 설계되었습니다 ( `uORB`, `parameter interface`, `dataman`, `perf` 도 변경되어야 합니다).

모듈을 실행하는 2가지 방법이 있습니다.

- **Tasks**: 모듈의 자신의 태스크, 스택, 프로세스 우선수위를 갖고 실행될 수 있습니다 (일반적인 방법입니다). 
- **Work queues**: 모듈은 자신의 스택은 가지지 않으면서 공유된 태스크에서 실행될 수 있습니다. Work queue 에서 같은 스택내의 다수의 태스크가 우선순위를 갖고 실행됩니다.
    
    하나의 태스크는 미래의 특정 시간으로 스케쥴링 됩니다. 이것의 장점은 RAM을 적게 사용하지만, sleep이나 메세지 poll을 허용하지 않습니다.
    
    Work queue는 센서 드라이버나 land detector와 같은 주기적은 태스크를 위해 사용됩니다.

> **Note** work queue에서 수행중인 태스크는 `top` 명령어에서 보여지지 않습니다 ( `lpwork` 같은 명령에서만 보여집니다).

### Background Tasks

`px4_task_spawn_cmd()` 는 호출하는 (부모) 태스크와 독립적으로 수행되는 새로운 태스크 (NuttX) 나 쓰레드 (POSIX - Linux/macOS) 를 시작할때 사용됩니다.

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

### OS-Specific Information

#### NuttX

[NuttX](http://nuttx.org/) 는 기체 제어 보드에서 PX4를 구동하는 주된 RTOS입니다. 오픈소스 (BSD license) 이며, 가볍고, 효율적이며 안정적입니다.

모듈들은 태스크로 실행됩니다. 그들은 자신의 파일 디스크립터 리스트를 가지지만, 하나의 주소 공간을 공유합니다. 태스크는 파일 디스크립터 리스트를 공유하면서 하나 이상의 스레드를 시작할 수 있습니다.

각각의 태스크/스레드는 고정된 크기의 스택을 갖고 있습니다. 그리고 모든 스택이 적당한 여유 공간을 갖고 있는지 검사하는 주기적인 태스크가 있습니다 (stack coloring 에 기초).

#### Linux/macOS

리눅스나 macOS에서는 PX4는 하나의 프로세스 안에서 동작합니다. 그리고 각 모듈들은 자신의 스레드를 갖고 수행됩니다 (NuttX의 태스크와 스레드간에는 특별한 차이는 없습니다).