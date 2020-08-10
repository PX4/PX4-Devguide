# 텔레메트리 무선 장비/모뎀

텔레메트리 무선 장치는 (별도로) *QGroundControl*같은 지상 통제국과 PX4를 구동하는 기체간 무선 MAVLink 연결 수단을 제공할 목적으로 활용합니다. 이 절에서는 지원 무선 장치의 고급 활용법 및 PX4로의 새 텔레메트리 시스템 통합 방법을 다룹니다.

## 지원 무선 장비 시스템

[PX4 사용자 안내서 > 텔레메트리](https://docs.px4.io/master/en/telemetry/)에서는 PX4를 지원하는 텔레메트리 무선 장비 시스템 정보가 들어있습니다. 게다가 이 절에는 *SiK 무선 장비* 펌웨어와 *3DR WiFi 텔레메트리 무선 장비* 활용법이 들어있습니다.

## 텔레메트리 시스템 통합

PX4 에서는 픽스호크 기반 비행체 조종기의 텔레메트리 포트로 MAVLink 기반 텔레메트리를 활용할 수 있습니다. 제공한 텔레메트리 무선 장치에서 MAVLink를 지원하며, 호환 전압 레벨/커넥터를 지원하는 UART 인터페이스가 있어 더이상의 통합 지원이 필요치 않습니다.

Telemetry systems that communicate using some other protocol will need more extensive integration, potentially covering both software (e.g. device drivers) and hardware (connectors etc.). While this has been done for specific cases (e.g. [FrSky Telemetry](https://docs.px4.io/master/en/peripherals/frsky_telemetry.html) enables sending vehicle status to an RC controller via an FrSky receiver) providing general advice is difficult. We recommend you start by [discussing with the development team](../README.md#support).