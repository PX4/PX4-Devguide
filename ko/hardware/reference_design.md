!REDIRECT "https://docs.px4.io/master/ko/hardware/reference_design.html"

# PX4 참조 비행체 제어 장치 설계

PX4 참조 설계는 비행체 제어 장치의 [픽스호크 계열](https://docs.px4.io/master/en/flight_controller/pixhawk_series.html)입니다. 2011년 처음 출시했으며, 현재 설계는 5번째 [세대](#reference_design_generations)입니다(6세대 보드 설계는 진행중).

## 바이너리 호환성

제각각의 설계에 따라 제조한 모든 보드는 잠재 바이너리 호환성을 지니고 있습니다(예: 동일한 펌웨어 실행 가능). 2018년도부터는 호환성을 검증하고 인증하는 바이너리 호환성 시험 기반을 제공합니다.

FMU 1~3 세대는 공개 하드웨어로 설계했으나, 4~5세대에서는 핀 출력과 전원 공급장치 명세 정보만 제공합니다(설계도는 각 제조사에서 만듦). 좀 더 우수한 호환성을 확보하기 위해, FMUv6 및 이후 버전에서는 완전한 참조 설계 모델로 돌아올 예정입니다.

<a id="reference_design_generations"></a>

## Reference Design Generations

* FMUv1: 개발 보드 \(STM32F407, 128 KB RAM, 1MB flash, [설계도](https://github.com/PX4/Hardware/tree/master/FMUv1)\) (PX4에서 더이상 지원하지 않음)
* FMUv2: 픽스호크 \(STM32F427, 168 MHz, 192 KB RAM, 1MB flash, [설계도](https://github.com/PX4/Hardware/tree/master/FMUv2)\)
* FMUv3: 2MB 플래시를 장착한 픽스호크 변형 버전 \(3DR 픽스호크 2 \(Solo\), Hex 픽스호크 2.1, Holybro Pixfalcon, 3DR Pixhawk Mini, STM32F427, 168 MHz, 256 KB RAM, 2 MB flash, [설계도](https://github.com/PX4/Hardware/tree/master/FMUv3_REV_D)\)
* FMUv4: 픽스레이서 \(STM32F427, 168 MHz, 256 KB RAM, 2 MB flash, [핀 출력도](https://docs.google.com/spreadsheets/d/1raRRouNsveQz8cj-EneWG6iW0dqGfRAifI91I2Sr5E0/edit#gid=1585075739)\) 
* FMUv4 PRO: 드로텍 픽스호크 3 PRO \(STM32F469, 180 MHz, 384 KB RAM, 2 MB flash, [핀 출력](https://docs.google.com/spreadsheets/d/1raRRouNsveQz8cj-EneWG6iW0dqGfRAifI91I2Sr5E0/edit#gid=1585075739)\)
* FMUv5: 홀리브로 픽스호크 4 \(STM32F765, 216 MHz, 512 KB RAM, 2 MB flash, [핀 아웃](https://docs.google.com/spreadsheets/d/1-n0__BYDedQrc_2NHqBenG1DNepAgnHpSGglke-QQwY/edit#gid=912976165)\)
* FMUv6: 개발 중, 최종 명칭 미정, 6s 변형 \(STM32H7, 400 MHz, 2 MB RAM, 2 MB flash\)과 6i 변형 \(i.MX RT1050, 600 MHz, 512 KB RAM, 외장 플래시\)

## 주요/입출력 기능 해부

The diagram below shows the division of bus and functional responsibilities between the FMU and I/O boards in a Pixhawk-series flight controller (the boards are incorporated into a single physical module).

![PX4 Main/IO Functional Breakdown](../../assets/diagrams/px4_fmu_io_functions.png)

<!-- Draw.io version of file can be found here: https://drive.google.com/file/d/1H0nK7Ufo979BE9EBjJ_ccVx3fcsilPS3/view?usp=sharing -->

Some Pixhawk-series controllers are built without the I/O board in order to reduce space or complexity, or to better address certain board use-cases.

The I/O board is disabled by setting parameter [SYS_USE_IO=0](../advanced/parameter_reference.md#SYS_USE_IO). When the I/O board is disabled:

- 메인 믹서 파일을 FMU로 불러옵니다(따라서 "메인" 출력은 [에어프레임 참고](../airframes/airframe_reference.md)에서 AUX로 표기한 포트로 나타냅니다). AUX 믹서 파일은 불러오지 않으므로, 이 파일에 지정한 출력 핀은 사용하지 않습니다.
- RC 입력은 입출력 보드가 아닌 FMU로 바로 들어갑니다.

Flight controllers without an I/O board have `MAIN` ports, but they *do not* have `AUX` ports. Consequently they can only be used in [airframes](../airframes/airframe_reference.md) that do not use `AUX` ports, or that only use them for non-essential purposes (e.g. RC passthrough). They can be used for most multicopters and *fully* autonomous vehicles (without a safety pilot using RC control), as these typically only use `MAIN` ports for motors/essential controls.

> **Warning** 입출력 보드가 빠진 비행체 제어 장치는 핵심 비행 제어부와 모터를 `AUX`포트에 연결하는 [에어프레임](../airframes/airframe_reference.md)에서 사용할 수 없습니다(`AUX` 포트가 없기 때문).

<span></span>

> **Note** 입출력 보드가 빠진 제조사의 비행체 제어 장치 변경 버전은 보통 해당 버전의 "소형" 모델로, 예를 들면, *픽스호크 4* **미니**_, *CUAV v5 **나노***와 같이 표기합니다.

Most PX4 PWM outputs are mapped to either `MAIN` or `AUX` ports in mixers. A few specific cases, including camera triggering and Dshot ESCs, are directly mapped to the FMU pins (i.e. they will output to *either* `MAIN` or `AUX`, depending on whether or not the flight controller has an I/O board).