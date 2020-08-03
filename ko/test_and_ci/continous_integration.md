# PX4 지속 통합

PX4는 다중 지속 통합 서비스로 빌드와 테스트 과정을 확장할 수 있습니다.

## [Travis-ci](https://travis-ci.org/PX4/Firmware)

Travis-ci는 [QGroundControl](http://qgroundcontrol.com/)을 통해 플래싱할 수 있는 바이너리를 공식 안정/베타/개발 버전으로 빌드할 수 있습니다. 도커 이미지 [px4io/px4-dev-base](https://hub.docker.com/r/px4io/px4-dev-base/) 에서는 GCC 4.9.3을 사용하며 makefile 타켓 qgc_firmware로 px4fmu-{v2, v4}, mindpx-v2, tap-v1 펌웨어를 컴파일합니다.

Travis-CI는 테스트를 동반한 macOS px4_sitl 빌드도 가능합니다.

## [세마포어](https://semaphoreci.com/px4/firmware)

세마포어는 퀄컴 스냅드래곤 플랫폼의 변경 내역을 찾아 컴파일하는데 주로 사용하지만 동일한 [px4io/px4-dev-base](https://hub.docker.com/r/px4io/px4-dev-base/) 도커 이미지를 활용한 Travis-CI의 백업으로도 활용 가능합니다. 게다가 Travis-CI로 컴파일하는 펌웨어 뿐만 아니라, 단위 테스트를 수행하고 코드 형식을 검증하는 stm32discovery, crazyfile 빌드도 수행합니다.

## [CircleCI](https://circleci.com/gh/PX4/Firmware)

CircleCI tests the proposed next version of GCC to be used for stable firmware releases using the docker image [px4io/px4-dev-nuttx-gcc_next](https://hub.docker.com/r/px4io/px4-dev-nuttx-gcc_next/). It uses the makefile target `quick_check` which compiles `px4_fmu-v4_default`, `px4_sitl_default`, runs testing, and verifies code style.