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
  * A board-specific initialisation file is automatically included in startup scripts if found under the boards directory at **init/rc.board**.
  * The file is used to start sensors (and other things) that only exist on a particular board. It may also be used to set a board's default parameters, UART mappings, and any other special cases.
  * For FMUv5 you can see all the Pixhawk 4 sensors being started, and it also sets a larger LOGGER_BUF. 

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

The PX4 project supports and maintains the [FMU standard reference hardware](../hardware/reference_design.md) and any boards that are compatible with the standard. This includes the [Pixhawk-series](https://docs.px4.io/master/en/flight_controller/pixhawk_series.html) (see the user guide for a [full list of officially supported hardware](https://docs.px4.io/master/en/flight_controller/)).

Every officially supported board benefits from:

* PX4 Port available in the PX4 repository
* Automatic firmware builds that are accessible from *QGroundControl*
* Compatibility with the rest of the ecosystem
* Automated checks via CI - safety remains paramount to this community
* [Flight testing](../test_and_ci/test_flights.md)

We encourage board manufacturers to aim for full compatibility with the [FMU spec](https://pixhawk.org/). With full compatibility you benefit from the ongoing day-to-day development of PX4, but have none of the maintenance costs that come from supporting deviations from the specification.

> **Tip** Manufacturers should carefully consider the cost of maintenance before deviating from the specification (the cost to the manufacturer is proportional to the level of divergence).

We welcome any individual or company to submit their port for inclusion in our supported hardware, provided they are willing to follow our [Code of Conduct](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/CODE_OF_CONDUCT.md) and work with the Dev Team to provide a safe and fulfilling PX4 experience to their customers.

It's also important to note that the PX4 dev team has a responsibility to release safe software, and as such we require any board manufacturer to commit any resources necessary to keep their port up-to-date, and in a working state.

If you want to have your board officially supported in PX4:

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