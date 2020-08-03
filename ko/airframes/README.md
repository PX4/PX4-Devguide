# 에어프레임

PX4에는 상상 가능한 비행체 형식/프레임을 단일 코드베이스로 지원하게 하는 유연한 [믹싱 시스템](../concept/mixing.md)을 채용했습니다:

* **비행기:** 일반 항공기, 전익 항공기, 역방향 V형 미익 항공기 등 
* **멀티콥터:** 헬리콥터, 트라이콥터, 쿼드콥터, 헥사로터, 도데카로터 등의 여러 기하적 프로펠러 배치.
* **수직 이착륙기:** VTOL 설정에 포함: 테일 시터, 틸트로터, 쿼드플레인(플레인 + 쿼드).
* **UGVs/Rovers:** 수동, 임무기반 제어를 가능케 하는 무인 지상 항공기를 대상으로 기본 지원 추가.

[에어프레임 참조](../airframes/airframe_reference.md)에서 지원하는 모든 프레임 형식과 모터 출력 형태 목록을 찾을 수 있습니다.

이 절에서는 새 비행체 또는 PX4 형식의 비행체 기능 지원을 추가하려는 개발자 관련 정보를 언급합니다. 여전히 개발 중인 비행체 빌드 로그 기능도 다룹니다.

> **Tip** PX4 또한 다른 비행체 형식과 일반 로봇, 잠수함, 보트, 수륙 양용 운송 수단을 포함한 시험 항공 장비에서 로켓까지 거의 모든 분야의 용도로 안성맞춤입니다. *Let us know* if you have a new vehicle or frame-type you want to help support in PX4.

<span></span>

> **Note** Build logs for some of the supported airframes can be found in [PX4 User Guide > Airframes](https://docs.px4.io/master/en/airframes/).