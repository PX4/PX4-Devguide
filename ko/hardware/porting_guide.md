# 비행체 조종 장치 이식 안내

이 주제에서는 *새* 비행체 조종 하드웨어가 PX4와 동작하도록 하려는 개발자에 해당하는 내용을 다룹니다.

## PX4 구조

PX4는 두개의 주요 계층으로 이루어져있습니다. 호스트 운영체제(NuttX, Linux 또는 Mac OS 유사 POSIX 계열 플랫폼)와 프로그램([src/modules](https://github.com/PX4/Firmware/tree/master/src/modules)의 플라이트 스택) 상단의 [보드 지원 계층과 미들웨어 계층입니다.](../middleware/README.md) 자세한 내용은 [PX4 구조 개요](../concept/architecture.md)를 참고하십시오.

이 안내서에서는 프로그램/플라이트 스택이 어떤 보드에서든 동작하므로 호스트 운영체제와 미들웨어에만 집중하겠습니다.

## 비행체 조종 장치 설정 파일 구조

보드의 시작, 설정 파일은 각 보드의 제조사별 디렉터리의 [/boards](https://github.com/PX4/Firmware/tree/master/boards/)에 있습니다(예: **boards/*VENDOR*/*MODEL*/**).

예를 들어, FMUv5에서는:

* 보드에 해당하는 (모든) 파일: [/boards/px4/fmu-v5](https://github.com/PX4/Firmware/tree/{{ book.px4_version }}/boards/px4/fmu-v5). 
* 빌드 설정: [/boards/px4/fmu-v5/default.cmake](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/boards/px4/fmu-v5/default.cmake).
* 개별 보드용 초기 파일: [/boards/px4/fmu-v5/init/rc.board_defaults](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/boards/px4/fmu-v5/init/rc.board_defaults) 
  * 개별 보드용 초기화 파일은 보드 디렉터리의 **init/rc.board** 경로에서 시작 스크립트를 찾으면 자동으로 넣습니다.
  * 개별 보드에 붙어있는 센서(및 기타 소자)를 시작할 때 사용하는 파일입니다. 이 파일은 보드 기본 매개변수 설정, UART 대응, 기타 특별한 경우에 활용합니다.
  * FMUv5에서는 픽스호크의 모든 센서를 시작하고 큰 LOGGER_BUF 값의 설정을 확인할 수 있습니다. 

## 호스트 운영체제 설정

이 절에서는 각 지원 운영체제를 새 비행체 조종 하드웨어에 이식할 목적으로 필요한 설정 파일의 목적과 위치를 설명합니다.

### NuttX

[NuttX 보드 포팅 안내](porting_guide_nuttx.md)를 살펴보십시오. 

### Linux

리눅스 보드에는 운영체제와 커널 설정이 들어있지 않습니다. 해당 보드에 맞는 리눅스 이미지를 이미 제공하고 있습니다(관성 센서 별도 지원에 필요함).

* [boards/px4/raspberrypi/default.cmake](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/boards/px4/raspberrypi/default.cmake) - RPI 교차 컴파일. 

## 미들웨어 요소와 설정

이 절에서는 다양한 미들웨어 요소와 새 비행체 조종 하드웨어에 이식할 설정 파일 업데이트를 설명합니다.

### QuRT / Hexagon

* 시작 스크립트는 [posix-configs/](https://github.com/PX4/Firmware/tree/{{ book.px4_version }}/posix-configs) 경로에 있습니다.
* 운영체제 설정은 기본 리눅스 이미지의 일부입니다 (처리할 일: 리눅스 이미지 위치와 플래싱 방법 안내).
* PX4 미들웨어 설정은 [src/boards](https://github.com/PX4/Firmware/tree/{{ book.px4_version }}/boards)에 있습니다. (처리할 일: 버스 설정 내용 추가) 
* 참고 설정: `make eagle_default`를 실행하면 스냅드래곤 플라이트 참조 설정을 빌드합니다.

## 원격 조종 UART 연결 추천

보통 원격 조종 장치를 연결하는 추천 방법은 마이크로 컨트롤러로의 RX 핀과 TX 핀 개별 연결 방식입니다. RX와 TX를 함께 연결하면 UART 통신 방식상 경합 현상을 피하기 위해 단일 회선 모드로 설정해야합니다. 이 문제는 보드와 mainfest 파일의 설정으로 해결할 수 있습니다. 예를 하나 들자면 [px4fmu-v5](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/boards/px4/fmu-v5/src/manifest.c)가 있습니다.

## 공식 지원 하드웨어

PX4 프로젝트에서는 [FMU 표준 참조 하드웨어](../hardware/reference_design.md) 와 표준 호환 보드를 지원하고 관리합니다. 이 범위에는 [픽스호크 계열](https://docs.px4.io/master/en/flight_controller/pixhawk_series.html)이 들어있습니다([공식적으로 지원하는 하드웨어 총 목록](https://docs.px4.io/master/en/flight_controller/)은 사용자 안내서에서 찾아보십시오).

모든 공식 지원 보드를 활용하면 다음과 같은 장점이 있습니다:

* PX4 저장소의 PX4 포팅 소스 코드 활용
* *QGroundControl*에서 접근할 수 있는 펌웨어 자동 빌드
* 생태계 호환성 확보
* 지속 통합 관리 시스템으로 자동 검사 - 이 커뮤니티에 있어 최고의 안전성 확보
* [비행체 시험](../test_and_ci/test_flights.md)

[FMU 명세](https://pixhawk.org/)에 최대한의 호환성을 확보한 보드 제조사의 제품 활용을 권합니다. 완전한 호환성을 갖춘 보드를 활용하면 PX4 일일 개발 소스코드로부터 여러가지 잇점을 취할 수 있으며, 사양으로부터 벗어나는 지원에 그 어떤 비용도 발생하지 않습니다.

> **Tip** 제조사에서는 기본 사양에 벗어나는 무언가를 추가할 때 유지 비용을 주의깊게 고려해야합니다(제조사의 비용은 다양한 정도에 비례합니다).

개인 또는 업체에서 자신들의 지원 하드웨어로의 소프트웨어 이식을 보내준다면 기꺼이 받도록 하겠습니다. 또한 기여 개인 또는 조직이 [활동 지침](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/CODE_OF_CONDUCT.md)을 따르며, 고객들에게 안전하고 만족스러운 PX4 경험을 안겨주기 위해 개발팀과 함께 일해주시면 좋겠습니다.

참고할 중요한 점이 또 있다면, PX4 개발팀은 안전한 프로그램을 출시할 책임이 있기에 보드 제조사에 항상 최신을 유지하는데 필요한 여러 지원 요소와 작업 현황을 알려달라고 요청합니다.

PX4에서 공식적으로 지원하고자 하는 보드가 있다면:

* Your hardware must be available in the market (i.e. it can be purchased by any developer without restriction).
* Hardware must be made available to the PX4 Dev Team so that they can validate the port (contact <lorenz@px4.io> for guidance on where to ship hardware for testing).
* The board must pass full [test suite](../test_and_ci/README.md) and [flight testing](../test_and_ci/test_flights.md).

**The PX4 project reserves the right to refuse acceptance of new ports (or remove current ports) for failure to meet the requirements set by the project.**

You can reach out to the core developer team and community on the official [Forums and Chat](../README.md#support).

## 관련 정보

* [장치 드라이버](../middleware/drivers.md) - 새 주변기기 하드웨어 (장치 드라이버) 지원 방법
* [코드 빌드](../setup/building_px4.md) - 펌웨어 소스 코드 빌드 및 업로드 방법 
* 지원 비행 조종기: 
  * [오토파일럿 하드웨어](https://docs.px4.io/master/en/flight_controller/) (PX4 사용자 안내서)
  * [지원 보드 목록](https://github.com/PX4/Firmware/#supported-hardware) (Github)
* [지원 주변기기](https://docs.px4.io/master/en/peripherals/) (PX4 사용자 안내서)