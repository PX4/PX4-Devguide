# PX4 아키텍처 개요

PX4는 2개의 주된 레이어로 구성됩니다. [flight stack](#flight-stack)은 비행제어를 추상화한 계층입니다. [middle](#middleware)는 자율 로봇, 내/외부 통신, 하드웨어 통합을 지원하는 일반적인 레이어입니다.

모든 PX4 [기체](../airframes/README.md)는 하나의 codebase (보트, 로봇, 잠수함 등의 다른 로보틱스 시스템도 포함) 를 공유합니다. 완전한 시스템 디자인은 [reactive](http://www.reactivemanifesto.org)입니다. 무슨 뜻이나면

- 모든 기능들은 교환가능하고 재사용가능한 컴포넌트들로 나뉩니다.
- 통신은 비동기적인 메시지 전달에 의해 수행됩니다.
- 이 시스템은 다양한 작업을 쉽게 다룰 수 있습니다.

## 고수준 소프트웨어 구조{#architecture}

아래 그림은 PX4를 구성하는 블럭 내용을 자세히 보여줍니다, 그림 최상부에는 미들웨어 부분이 있고, 그 아래 부분은 flight stack 부분이 있습니다.

![PX4 Architecture](../../assets/diagrams/PX4_Architecture.svg)

<!-- This diagram can be updated from 
[here](https://drive.google.com/file/d/0B1TDW9ajamYkaGx3R0xGb1NaeU0/view?usp=sharing) 
and opened with draw.io Diagrams. You might need to request access if you
don't have a px4.io Google account.
Caution: it can happen that after exporting some of the arrows are wrong. In
that case zoom into the graph until the arrows are correct, and then export
again. -->

소스 코드는 자체 모듈/프로그램으로 나눕니다(도표의 `monospace` 참고). 보통 블록 구성은 정확히 하나의 모듈에 대응합니다.

> **Tip** 실행 중에, 셸에서 `top` 명령으로 어떤 모듈을 실행하는지 볼 수 있으며, 어떤 모듈을 `<module_name> start/stop` 명령으로 제각각 시작하고 멈출 수 있는지 확인할 수 있습니다. `top` 명령어는 NuttX 쉘에서만 쓸 수 있지만, 다른 명령어들은 SITL 쉘(pxh>)에서도 사용할 수 있습니다. 이 모듈에 대한 더 많은 내용은 [모듈 & 명령 참고](../middleware/modules_main.md)를 참고하십시오. 

화살표는 모듈간의 *가장 중요한* 연결의 정보 흐름을 보여줍니다. 실제로는 그림에서 나타낸 연결 수보다 더 많고, 일부 데이터(예: 매개변수)는 상당히 많은 모듈에서 접근합니다.

각 모듈은 [uORB](../middleware/uorb.md)라고 하는 publish-subscribe 메세지 버스로 통신합니다. Publish-subscribe 스킴의 사용은 다음을 의미합니다.

- 반응형 시스템 — 비동기 방식으로 동작하며 새 데이터를 넣으면 바로 업데이트합니다
- 모든 연산, 통신 동작에 대해 완전한 병렬 처리를 수행합니다
- 시스템 구성요소는 스레드 처리에 안전한 방식으로 어디에서든 데이터를 활용할 수 있습니다.

> **Info** 이 아키텍처는 실행 시간중에도 모든 단일 블록을 빠르고 쉽게 대체하도록 합니다.

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

**컨트롤러**는 설정값, 측정장 또는 추정값 (측정된 값을 처리한 값) 을 입력으로 받아들이는 컴포넌트 입니다. 이 장치의 목표는 설정값과 일치하도록 처리 변수 값을 조정하는 것입니다. 출력은 설정값에 도달하기 위해 보정된 값입니다. 예를 들어, 위치 제어부는 위치 설정 값을 입력 받고, 처리 변수는 현재 추정 위치 값을 담으며, 자세 및 추력 설정값을 출력으로 내보내 원하는 위치로 기체를 옮깁니다.

**믹서**는 명령 (예. 우회전)을 받아들이고 해석하여 개별 모터 명령으로 변환하지만 한계 조건을 넘지 않습니다. 이 변환은 기체의 타입, 무게 중심을 기준으로한 모터의 배열, 기체의 회전 관성과 같은 요소들에 의존적입니다.

### 미들웨어 {#middleware}

[미들웨어](../middleware/README.md) 는 주로 임베디드 센서, 외부와의 통신 (보조 컴퓨터, GCS, 등), uORB publish-subscribe 메시지 버스용 장치 드라이버로 이루어져있습니다.

게다가, middleware는 [simulation layer](../simulation/README.md)를 포함합니다. simulation layer는 데스크탑 OS에서 PX4 코드의 실행과 가상으로 구현된 기체의 제어를 지원합니다.

## Update Rates

모듈들은 메시지를 업데이트를 기다리므로, 보통 드라이버는 모듈 업데이트 속도를 정의합니다. 대부분의 IMU 드라이버는 1kHZ로 데이터를 샘플링하고 합하여, 250Hz로 공표합니다. `navigator`와 같은 시스템의 다른 부분들은 빨리 업데이트할 필요가 없어서 상대적으로 느리게 수행합니다.

메시지의 업데이트 속도는 시스템의 `uORB top`에 의해 실시간으로 [검사](../middleware/uorb.md) 됩니다.

## 런타임 환경 {#runtime-environment}

PX4에서는 POSIX-API를 제공하는 다양한 운영체제(Linux, macOS, NuttX, QuRT)에서 동작합니다. 이 운영체제에는 실시간 스케쥴링(예: FIFO)같은 기능이 들어갑니다.

([uORB](../middleware/uorb.md)을 이용한) 모듈간 통신은 공유 메모리 기반입니다. PX4 미들웨어 전부는 단일 주소 공간에서 실행합니다. 예를 들면 메모리는 모든 모듈에서 공유합니다.

> **Info** 시스템은 개별 주소 공간에서 각 모듈을 실행하는데 최소한의 비용이 들어가도록 설계했습니다(`uORB`, `매개변수 인터페이스`, `dataman`, `perf` 같은 부분을 조금 바꿔야 합니다).

모듈을 실행하는 방법에는 2가지가 있습니다.

- **작업**: 모듈은 자체 스택을 확보하고 프로세스 우선순위를 부여받아 자체 작업내에서 실행합니다.
- **작업 큐(work queue)의 작업**: 모듈은 동일한 스택과 큐의 다른 모듈처럼 작업 큐 스레드 우선순위를 부여받은 공유 작업 큐에서 실행합니다.
  
  - 모든 작업은 서로의 동작을 중단하지 않고 사이좋게 동작해야합니다.
  - 다중 *작업 큐 작업*은 큐에서 실행할 수 있으며, 다중 큐가 될 수 있습니다.
  - *작업 큐의 작업*은 향후 지정 시간에 동작하도록 계획하거나 uORB 토픽 업데이트 콜백으로 처리합니다.
  
  실행 큐에서 모듈을 실행하는 장점은 RAM 소모량이 적고, 잠재적으로 작업 전환이 빈번하지 않는다는 점입니다. 단점은 *작업 큐 작업*을 대기 상태로 두거나 메시지를 폴링하거나 입출력을 멈출(파일 읽기 등) 수 없습니다. 장시간 실행 작업(막대한 양의 계산처리 수행)의 경우 잠재적으로 개별 작업으로 분리하여 실행하거나 최소한 작업 큐를 분할해야 합니다.

> **Note** 작업 큐에서 실행하는 작업은 [`uorb top`](../middleware/modules_communication.md#uorb)에 나타나지 않습니다(작업 큐 자체는 `wq:lp_default`처럼 나타날 수는 있습니다). 모든 활성 작업 큐 항목을 보려면 [`work_queue status`](../middleware/modules_system.md#workqueue) 명령을 활용하십시오.

### 백그라운드 작업

`px4_task_spawn_cmd()` 는 호출(상위) 작업으로부터 독립적으로 실행하는 새 작업 (NuttX) 또는 스레드(POSIX - Linux/MacOS) 실행에 활용합니다.

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

[NuttX](http://nuttx.org/) 는 기체 제어 보드에서 PX4를 구동하는 주된 RTOS입니다. 오픈소스 (BSD license) 이며, 가볍고, 효율적이며 안정적입니다.

모듈은 작업 처럼 실행합니다. 자체적으로 파일서술자 목록를 가지나, 단일 주소 공간을 공유합니다. 작업은 파일서술자 목록을 공유하는 하나 이상의 스레드를 시작할 수 있습니다.

각 작업/스레드는 고정 크기 스택을 가지며, 모든 스택에 충분한 여분의 공간이 남아있는지 검사하는 (스택 콜로닝 기반) 주기적인 작업이 있습니다.

#### Linux/macOS

리눅스나 macOS에서는 PX4는 하나의 프로세스 안에서 동작합니다. 그리고 각 모듈들은 자신의 스레드를 갖고 수행됩니다 (NuttX의 태스크와 스레드간에는 특별한 차이는 없습니다).