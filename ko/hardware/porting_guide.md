# 비행체 조종기 이식 안내

이 주제에서는 *새* 비행체 조종 하드웨어가 PX4와 동작하도록 하려는 개발자에 해당하는 내용을 다룹니다.

## PX4 구조

PX4는 두개의 주요 계층으로 이루어져있습니다. 호스트 운영체제(NuttX, Linux 또는 Mac OS 유사 POSIX 계열 플랫폼)와 프로그램([src/modules](https://github.com/PX4/Firmware/tree/master/src/modules)의 플라이트 스택) 상단의 [보드 지원 계층과 미들웨어 계층입니다.](../middleware/README.md) 자세한 내용은 [PX4 구조 개요](../concept/architecture.md)를 참고하십시오.

This guide is focused only on the host OS and middleware as the applications/flight stack will run on any board target.

## Flight Controller Configuration File Layout

Board startup and configuration files are located under [/boards](https://github.com/PX4/Firmware/tree/master/boards/) in each board's vendor-specific directory (i.e. **boards/*VENDOR*/*MODEL*/**)).

For example, for FMUv5:

* (All) Board-specific files: [/boards/px4/fmu-v5](https://github.com/PX4/Firmware/tree/{{ book.px4_version }}/boards/px4/fmu-v5). 
* Build configuration: [/boards/px4/fmu-v5/default.cmake](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/boards/px4/fmu-v5/default.cmake).
* Board-specific initialisation file: [/boards/px4/fmu-v5/init/rc.board_defaults](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/boards/px4/fmu-v5/init/rc.board_defaults) 
  * A board-specific initialisation file is automatically included in startup scripts if found under the boards directory at **init/rc.board**.
  * The file is used to start sensors (and other things) that only exist on a particular board. It may also be used to set a board's default parameters, UART mappings, and any other special cases.
  * For FMUv5 you can see all the Pixhawk 4 sensors being started, and it also sets a larger LOGGER_BUF. 

## Host Operating System Configuration

This section describes the purpose and location of the configuration files required for each supported host operating system to port them to new flight controller hardware.

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

It is generally recommended to connect RC via separate RX and TX pins to the microcontroller. If however RX and TX are connected together, the UART has to be put into singlewire mode to prevent any contention. This is done via board config and manifest files. One example is [px4fmu-v5](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/boards/px4/fmu-v5/src/manifest.c).

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