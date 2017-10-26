# Airframes

PX4는 유연한 믹싱 시스템을 가지고 있어서 단일 코드베이스로 우리가 생각할 수 있는 거의 대부분의 vehicle 타입/프레임을 지원합니다. :

* **Planes:** 일반 비행기, 비행 날개, V-tail 전환 비행체 등
* **VTOL Airframes:** VTOL 설정은 다음을 포함: Tailsitters, Tiltrotors 그리고 QuadPlanes (plane + quad)
* **UGVs/Rovers:** 무인 지상 vehicle 및 수동과 미션 기반 제어에 대해서 기본 지원

지원하는 airframes에 대해서 빌드 로그는 [PX4 User Guide](https://docs.px4.io/en/airframes/)에서 참고할 수 있습니다. 지원하는 모든 프레임 타입과 모터 출력의 목록도 [Airframes Reference](../airframes/airframe_reference.md)에서 참고할 수 있습니다.

> **Note** airframe 빌드 로그는 사용자 가이드로 옮기고 있습니다.

PX4는 잠수정, 보트, 수륙양용과 같이 여러 vehicle 타입과 일반적인 로봇에 사용이 가능하며 실험적으로 항공기나 로켓에도 사용ㅎ고 있습니다.

이 섹션에서는 PX4에 새로운 vehicle 타입을 추가하기를 원하는 개발자에게 정보를 제공합니다. 현재 개발 중인 vehicle에 대한 빌드 로그도 포함되어 있습니다.

> **Tip** 만약 여러분이 PX4에 지원 도움이 필요한 새로운 vehicle이나 프레임 타입을 가지고 있다면 알려주세요.
