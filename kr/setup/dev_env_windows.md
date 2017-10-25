# Windows 설치 방법

> **Warning** 윈도우 툴체인이 있긴하지만, 공식적으로 지원되는 것이 아니라 사용을 추천하지 않습니다. 펌웨어를 컴파일에 드는 시간이 오래 걸리고 Snapdragon Flight 같은 새로운 보드에 대해서 지원하지 않습니다. 많은 개발자가 컴퓨터 비전과 네비게이션 프로토타입으로 사용하는 표준 ROS 패키지를 사용할 수 없습니다. Windows에서 개발하기 전에, [Ubuntu](http://ubuntu.com)로 듀얼부트되는 환경을 고려해 보세요.

## 개발 환경 설치

시스템에 다운로드 및 설치 :

  * [Qt Creator IDE](http://www.qt.io/download-open-source/#section-6)
  * [PX4 Toolchain Installer v14 윈도우용 다운로드](http://firmware.diydrones.com/Tools/PX4-tools/px4_toolchain_installer_v14_win.exe) (32/64 bit 시스템, 완전한 빌드 시스템, 드라이버)
  * [PX4 USB Drivers](http://pixhawk.org/static/px4driver.msi) (32/64 bit 시스템)

이제 [처음 빌드하기](../setup/building_px4.md)를 이어서 실행하면 됩니다!

## 새소식! 윈도우용 Bash

Linux 빌드 명령을 사용할 수 있는 Bash shell 사용할 수 있게 되면서 Windows 사용자에게 새로운 선택사항이 생겼습니다. [BashOnWindows](https://github.com/Microsoft/BashOnWindows)를 참고하세요. 이 환경에서 px4 빌드가 성공적으로 되는 것을 확인하였습니다. 아직 펌웨어를 flash할 수는 없지만 Mission Planner나 QGroundControl을 사용하면 Windows에서도 커스텀 펌웨어를 플래쉬할 수 있습니다.

Note: Pixhawk ARM 펌웨어를 빌드하려면 [64 bit arm-none-eabi 컴파일러](https://github.com/SolinGuo/arm-none-eabi-bash-on-win10-.git)를 사용해야 합니다. BashOnWindows은 32 bit ELF 프로그램을 실행하지 않고 `https://launchpad.net/gcc-arm-embedded`의 기본 컴파일러는 32bit입니다.
따라서 SolinGuo가 생성한 *.tar.bz2 파일을 다운로드 받고 BashOnWindows 콘솔에서 다음 명령을 실행해서 압축을 풉니다 :

`tar -xvf gcc-arm-none-eabi-5_4-2017q2-20170512-linux.tar.bz2`

arm gcc 크로스 컴파일러를 포함하는 다음 폴더가 생깁니다 :

`gcc-arm-none-eabi-5_4-2017q2/bin`

이 폴더를 PATH에 추가할려면 보통 export PATH=...를 사용하면 PX4 빌드는 이 컴파일러를 찾아서 실행할 수 있습니다. 다음으로 `make px4fmu-v2_default`을 BashOnWindows에서 실행하면 펌웨어가 생기게 됩니다 : `build/px4fmu-v2_default/src/firmware/nuttx/px4fmu-v2_default.px4`. 이제 이 펌웨어를 QGroundControl을 이용해서 Pixhawk에 플래쉬합니다.
