# PX4 지속통합(Continuous Integration)

PX4 빌드와 테스팅은 여러 지속통합 서비스를 사용합니다.

## [Travis-ci](https://travis-ci.org/PX4/Firmware)

Travis-ci는 [QGroundControl](http://qgroundcontrol.com/)로 플래쉬할 수 있는 공식적으로 stable/beta/development 바이너리에 관한 책임을 지고 있습니다. 현재 docker 이미지 [px4io/px4-dev-base](https://hub.docker.com/r/px4io/px4-dev-base/)에 포함된 GCC 4.9.3을 사용하며 px4fmu-{v1, v2, v4}, mindpx-v2, tap-v1 에 대해 makefile target qgc_firmware로 컴파일합니다.

Travis-ci는 테스팅을 포함한 MacOS posix sitl 빌드가 있습니다.

## [Semaphore](https://semaphoreci.com/px4/firmware)

주로 Qualcomm Snapdragon 플랫폼의 변경을 컴파일하는데 사용합니다. 하지만 동일한 [px4io/px4-dev-base](https://hub.docker.com/r/px4io/px4-dev-base/) docker 이미지를 사용해서 Travis-ci를 백업하는 역할을 합니다. Travis-ci로 컴파일되는 펌웨어의 집합에 추가로 Semaphore도 stm32discovery, crazyflie용으로 빌드되고 단위테스트를 실행하고 코드 스타일을 검증합니다.

## [CircleCI](https://circleci.com/gh/PX4/Firmware)

CircleCI는 [px4io/px4-dev-nuttx-gcc_next](https://hub.docker.com/r/px4io/px4-dev-nuttx-gcc_next/) docker 이미지를 사용해서 릴리즈되는 안정 펌웨어에 사용할 GCC의 차기 버전을 테스트합니다. makefile target quick_check를 사용해서 px4fmu-v4_default, posix_sitl_default을 컴파일하고 테스팅을 실행하고 코드 스타일을 검증합니다.
