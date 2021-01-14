!REDIRECT "https://docs.px4.io/master/ko/hardware/porting_guide_nuttx.html"

# NuttX 보드 이식 안내

새 대상 하드웨어에 NuttX용 PX4를 이식하려면, 해당 대상 하드웨어가 NuttX에서 지원해야합니다. NuttX 프로젝트는 NuttX를 새 처리 플랫폼에 이식하는 최상의 [이식 안내서](https://cwiki.apache.org/confluence/display/NUTTX/Porting+Guide)를 관리하고 있습니다.

The following guide assumes you are using an already supported hardware target or have ported NuttX (including the [PX4 base layer](https://github.com/PX4/PX4-Autopilot/tree/master/platforms/nuttx/src/px4)) already.

The configuration files for all boards, including linker scripts and other required settings, are located under [/boards](https://github.com/PX4/PX4-Autopilot/tree/master/boards/) in a vendor- and board-specific directory (i.e. **boards/_VENDOR_/_MODEL_/**)).

다음 예제는 NuttX 기반 비행체 제어 장치 용도로 최신 [기준 참고 설정](../hardware/reference_design.md)인 FMUv5를 활용합니다:
* Running `make px4_fmu-v5_default` from the **PX4-Autopilot** directory will build the FMUv5 config
* The base FMUv5 configuration files are located in: [/boards/px4/fmu-v5](https://github.com/PX4/PX4-Autopilot/tree/master/boards/px4/fmu-v5).
  * Board specific header (NuttX pins and clock configuration): [/boards/px4/fmu-v5/nuttx-config/include/board.h](https://github.com/PX4/PX4-Autopilot/blob/master/boards/px4/fmu-v5/nuttx-config/include/board.h).
  * Board specific header (PX4 configuration): [/boards/px4/fmu-v5/src/board_config.h](https://github.com/PX4/PX4-Autopilot/blob/master/boards/px4/fmu-v5/src/board_config.h).
  * NuttX OS config (created with NuttX menuconfig): [/boards/px4/fmu-v5/nuttx-config/nsh/defconfig](https://github.com/PX4/PX4-Autopilot/blob/master/boards/px4/fmu-v5/nuttx-config/nsh/defconfig).
  * Build configuration: [boards/px4/fmu-v5/default.cmake](https://github.com/PX4/PX4-Autopilot/blob/master/boards/px4/fmu-v5/default.cmake).

## NuttX 메뉴 설정을 통한 설치

NuttX 운영체제 설정을 수정하려면 PX4 단축 설정을 활용하여 [menuconfig](https://bitbucket.org/nuttx/nuttx) 를 실행할 수 있습니다:
```sh
make px4_fmu-v5_default menuconfig
make px4_fmu-v5_default qconfig
```

For fresh installs of PX4 onto Ubuntu using [ubuntu.sh](https://github.com/PX4/PX4-Autopilot/blob/{{ book.px4_version }}/Tools/setup/ubuntu.sh) you will also need to install *kconfig* tools from [NuttX tools](https://bitbucket.org/nuttx/tools/src/master/).

> **Note** [px4-dev-nuttx](https://hub.docker.com/r/px4io/px4-dev-nuttx/) 도커 컨테이너나 공식적으로 안내하는 macOS용 일반 설치 절차를 따랐다면, (`kconfig-mconf`를 이미 가지고 있으므로) 이 단계는 굳이 필요하지 않습니다.

어떤 디렉터리에서든 다음 명령을 실행하십시오.
```sh
git clone https://bitbucket.org/nuttx/tools.git
cd tools/kconfig-frontends
sudo apt install gperf
./configure --enable-mconf --disable-nconf --disable-gconf --enable-qconf --prefix=/usr
make
sudo make install
```

`--prefix=/usr` 인자 값은 설치 위치를 지정합니다(`PATH` 환경 변수에 있어야 함). `--enable-mconf`와 `--enable-qconf` 옵션은 각각 `menuconfig`와 `qconfig` 옵션을 활성화합니다.

`qconfig`를 실행하려면 Qt 추가 의존 요소를 설치해야합니다.

### 부트로더

우선, 하드웨어 대상에 따른 부트로더가 필요합니다:
- STM32H7: NuttX 기반 부트로더이며, PX4 펌웨어에 들어있습니다. See [here](https://github.com/PX4/PX4-Autopilot/tree/master/boards/holybro/durandal-v1/nuttx-config/bootloader) for an example.
- 다른 대상용으로는, https://github.com/PX4/Bootloader를 사용합니다. 새 대상 추가 방법 예제를 보려면 [여기](https://github.com/PX4/Bootloader/pull/155/files)를 살펴보십시오. 그 다음 [빌드, 플래시 방법](../software_update/stm32_bootloader.md)을 살펴보십시오.

### 펌웨어 이식 절차

1. [development setup](../setup/dev_env.md)이 동작하는지 NuttX 메뉴 설정 도구를 설치했는지 확인하십시오(위 참고).
1. 소스 코드를 다운로드한 후 기존 대상에 빌드할 수 있는지 확인하십시오:
   ```bash
   git clone --recursive https://github.com/PX4/PX4-Autopilot.git
   cd PX4-Autopilot
   make px4_fmu-v5
   ```
1. 기존 대상과 동일한(또는 거의 근접한) CPU 형식을 찾아 복사하십시오. 예를 들면 STM32F7의 경우:
   ```bash
   mkdir boards/manufacturer
   cp -r boards/px4/fmu-v5 boards/manufacturer/my-target-v1
   ```
   **manufacturer** 부분은 실제 제조사 명칭으로 **my-target-v1**은 보드 이름으로 바꾸십시오.

다음 **boards/manufacturer/my-target-v1**에 모든 파일을 두고 해당 파일을 보드에 맞춰 업데이트하십시오.
1. **firmware.prototype**: 보드 ID와 이름을 업데이트 하십시오.
1. **default.cmake**: 디렉터리 이름(**my-target-v1**)과 일치하는 **VENDOR**, **MODEL** 값을 업데이트하십시오. (그다음) 직렬 포트를 설정하십시오.
1. `make manufacturer_my-target-v1 menuconfig` 명령으로 NuttX를 설정(**defconfig**) 하십시오. CPU, 칩 설정을 조정하고, 주변기기(UART, SPI, I2C, ADC)를 설정하십시오.
1. **nuttx-config/include/board.h**: NuttX 핀을 설정하십시오. NuttX에서 다중 핀 옵션을 지닌 모든 주변 장치가 어떤 핀에 붙었는지 알아야 합니다. 칩별 핀맵 헤더 파일에 정의 내용이 들어있습니다. 예를 들면, [stm32f74xx75xx_pinmap.h](https://github.com/PX4/NuttX/blob/px4_firmware_nuttx-8.2/arch/arm/src/stm32f7/hardware/stm32f74xx75xx_pinmap.h) 헤더 파일이 있습니다.
1. **src**: **src**에 모든 파일을 두고 필요한 설정, 특히 **board_config.h** 파일을 업데이트 하십시오.
1. **init/rc.board_sensors**: 보드에 붙은 센서를 시작하십시오.

