# 아키텍쳐 개요

PX4는 2개 주요 레이어로 구성 : autopilot 소프트웨어 솔루션인 [PX4 flight stack](../concept/flight_stack.md)과 자동 로봇을 지원할 수 있는 일반적인 로보틱스 미들웨어인 [PX4 미들웨어](../concept/middleware.md)로 구성됩니다.

모든 [airframes](../airframes/architecture.md)과 보트를 포함한 실제로 모든 로보틱스 시스템이 하나의 코드베이스를 공유합니다. 전체 시스템 설계 원칙은 [리액티브](http://www.reactivemanifesto.org) 특성을 가집니다. 그 의미는 :

  * 모든 기능이 교환가능한 컴포넌트들로 구성
  * 통신은 비동기 메시지 전달 방식
  * 시스템은 가변 workload를 처리가능

런타임 환경에 대한 고려 이외에도 모듈화로 [재사용성](https://en.wikipedia.org/wiki/Reusability)을 극대화시켰습니다.

## 상위레벨 소프트웨어 아키텍쳐

아래 각 블록들은 독립적 모듈로 코드, 의존성, 런타임 관점에서 독립적인 속성을 가집니다. 각 화살표는 [uORB](../middleware/uorb.md)를 통해 publish/subscribe 호출을 통해서 연결됩니다.

> ** Info ** PX4 아키텍쳐는 런타임에서도 빠르고 편리하게 각 블록 교환이 가능합니다.

컨트롤러/믹서는 airframe에 따라 지정합니다.(멀티콥터, VTOL, 비행기 타입) 하지만 `commander`와 `navigator` 같은 상위레벨 mission management 블록은 플랫폼 간에 공유가 가능하다.

![Architecture](../../assets/diagrams/PX4_Architecture.png)

> ** Info ** 여기 플로우 차트는 [여기](https://drive.google.com/file/d/0Byq0TIV9P8jfbVVZOVZ0YzhqYWs/view?usp=sharing)에서 업데이트되고 draw.io 다이아그램을 이용합니다.

## GCS를 이용하는 통신 아키텍쳐

GCS와 상호작용은 "business logic"을 통해 처리됩니다. 여기에는 commander(일반적인 명령과 제어로 예제로 arming), navigator(mission 받아서 이를 하위레벨의 기본적인 navigation으로 변환), mavlink application이 포함됩니다. MAVLink 패킷을 받아서 onboard uORB 자료구조로 변환합니다. 이렇게 분리하는 이유는 시스템내에서 MAVLink와 깊은 의존을 갖는 것을 명시적으로 방지하기 위한 구조입니다. MAVLink application은 많은 sensor 데이터를 사용하며, estimate를 사용하고 이를 ground control station으로 보냅니다.

[![](https://mermaid.ink/img/eyJjb2RlIjoiZ3JhcGggVEQ7XG4gIG1hdmxpbmstLS1jb21tYW5kZXI7XG4gIG1hdmxpbmstLS1uYXZpZ2F0b3I7XG4gIHBvc2l0aW9uX2VzdGltYXRvci0tPm1hdmxpbms7XG4gIGF0dGl0dWRlX2VzdGltYXRvci0tPm1hdmxpbms7XG4gIG1peGVyLS0-bWF2bGluazsiLCJtZXJtYWlkIjp7InRoZW1lIjoiZGVmYXVsdCJ9LCJ1cGRhdGVFZGl0b3IiOmZhbHNlfQ)](https://mermaid-js.github.io/mermaid-live-editor/#/edit/eyJjb2RlIjoiZ3JhcGggVEQ7XG4gIG1hdmxpbmstLS1jb21tYW5kZXI7XG4gIG1hdmxpbmstLS1uYXZpZ2F0b3I7XG4gIHBvc2l0aW9uX2VzdGltYXRvci0tPm1hdmxpbms7XG4gIGF0dGl0dWRlX2VzdGltYXRvci0tPm1hdmxpbms7XG4gIG1peGVyLS0-bWF2bGluazsiLCJtZXJtYWlkIjp7InRoZW1lIjoiZGVmYXVsdCJ9LCJ1cGRhdGVFZGl0b3IiOmZhbHNlfQ)
