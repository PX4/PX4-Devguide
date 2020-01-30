# 시스템 스타트업

PX4 스타트업은 쉘 스크립트에 의해 제어됩니다. 쉘 스크립트는 NuttX는 [ROMFS/px4fmu_common/init.d](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d) 폴더에 있습니다. 몇몇의 Posix 계열(Linux/MacOS)도 동일합니다. Posix만을 위한 스크립트는 [ROMFS/px4fmu_common/init.d-posix](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d-posix)에 위치합니다.

All files starting with a number and underscore (e.g. `10000_airplane`) are predefined airframe configurations. 설정값들은 빌드타임에 하나의 `airframes.xml`으로 보내지고 [QGroundControl](http://qgroundcontrol.com)을 통해 기체 선택 UI에 활용됩니다. 새로운 설정을 다루기 위해서는 [여기](../airframes/adding_a_new_frame.md)를 참고하세요.

남아 있는 파일들을 일반적인 스타트업 로직을 위해 사용됩니다. 처음 실행되는 파일은 [init.d/rcS](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d/rcS)로 ( 또는 Posix에서는 [init.d-posix/rcS](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d-posix/rcS)), 다른 모든 스크립트들을 호출합니다.

다음의 섹션은 PX4가 실행되는 운영체제에 따라 구분되어 있습니다.

## Posix (Linux/MacOS)

Posix에서는 시스템 쉘이 쉘 인터프리터로 사용됩니다 (예. /bin/sh는 우분에서 대시로 심볼릭링크 됨) 동작하기 위한 몇가지 조건들이 있습니다.

- PX4 모듈은 시스템에서 개별적으로 실행될 수 있어야합니다. 이것은 심볼릭 링크에 의해 수행됩니다. 각 모듈에 대해 심볼릭 링크 `px-4<module>-> px4`가 `bin` 디렉토리에 생성됩니다. 실행될 때, 바이너리의 경로가 (`argv[0]`) 확인되고, 만약 모듈이라면 (`px4-`로 시작), 메인 px4 인트턴스에게 명령을 보냅니다 (자세한건 아래로). **Tip** `px-4` 접두어는 시스템 명령어와의 충돌을 피하기 위해 사용됩니다 (예. `shutdown`), 그리고 `px-4<TAB>`를 타이핑함으로써 쉬운 탭 자동완성도 지원합니다.
- 쉘은 심볼릭 링크를 찾기위한 위치를 알아야합니다. 따라서 스타트업 스크립트를 실행하기전에 심볼릭 링크된 `bin` 디렉토리가 `PATH` 변수에 추가되어야합니다.
- 쉘은 각 모듈을 새로운 (클라이언트) 프로세스로 실행시킵니다. 메인 PX4 인스턴스 (서버)는 쓰레드로 동작하며, 각 클라이언트 프로세스는 메인 PX4와 통신할 수 있어야합니다. 통신은 [UNIX socket](http://man7.org/linux/man-pages/man7/unix.7.html)을 통해 이뤄집니다. 서버는 클라이언트가 연결할 수 있고 명령어를 보낼 수 있는 소켓을 수신합니다. 그러면 서버는 출력과 리턴 코드를 클라이언트에게 보냅니다.
- 스타트업 스크립트는 `px-4`로 시작하는 모듈이 대신 모듈을 직접적으로 호출합니다. 예. `commander start` 이것은 alias를 통해 수행됩니다. 각 모듈에 대해 `alias<module>=px4-<module>`의 형태로 alias가 `bin/px4-alias.sh`에 생성됩니다.
- `rcS` 스크립트는 메인 PX4 인스턴스에 의해 실행됩니다. 이 스크립트는 다른 어떤 모듈들을 실행시키지 않습니다. `PATH` 변수를 업데이트하고 파라미터로 `rcS`을 실행시킵니다.
- 거기에 더해, 다중서버 인스턴스는 다중-기체 시뮬레이션을 위해 수행될 수 있습니다. 클라이언트는 `--instance`를 통해 서버 인스턴스를 선택합니다. 그 인스턴스는 `$px4_instance` 변수를 통해 스크립트에서 이용가능합니다.

모듈은 PX4가 이미 실행중인 시스템이면 어느 터미널을 통해서든지 실행할 수 있습니다. 예:

    cd <Firmware>/build/px4_sitl_default/bin
    ./px4-commander takeoff
    ./px4-listener sensor_accel
    

### 동적 모듈

보통, 모든 모듈들은 하나의 PX4 실행파일에 컴파일 됩니다. 그러나 Posix에서는 모듈을 분리된 파일로 컴파일할 수 있는 옵션이 있습니다. `dyn` 명령을 통해 PX4로 로드할 수 있습니다.

    dyn ./test.px4mod
    

## NuttX

NuttX는 하나의 통합된 쉘 인터프리터 ([NSH](http://nuttx.org/Documentation/NuttShell.html))을 갖고 있습니다. 따라서 스크립트는 바로 실행될 수 있습니다.

### 시스템 부팅 디버깅

드라이버 하나의 실패가 부팅의 중단을 발생시키지는 않습니다. 이것은 스타트업 스크립트에서 `set +e`를 통해 제어됩니다.

부팅 순서는 [system console](../debug/system_console.md)을 통해 디버깅 할 수있습니다. 출력되는 부팅 로그는 부팅 순서에 대한 자세한 정보를 포함하고 왜 부팅이 중단 돼었는지는 포함해야 합니다.

#### 일반적인 부팅 실패 사례

- 커스텀 응용프로그램: 시스템의 메모리 사용량을 벗어납니다. `free` 명령을 수행해 사용가능한 RAM이 얼마인지 확인합니다.
- 스택 트레이스결과 소프트웨어 실패(fault)

### 시스템 스타트업 바꾸기

대부분의 경우에서 부팅을 커스터마이징하는 것이 더 나은 방법입니다. 만약 부팅은 바꿔야 한다면, miscroSD 카드의 `etc` 폴더에 위치한 `/fs/microsd/etc/rc.txt` 파일을 만들어야 합니다. 만약 그 파일에 아무것도 없다면 auto-start가 수행될 것입니다.

### 시스템 스타트업 커스터마이징하기

시스템 스타트업을 커스터마이징하기 위한 최선의 방법은 [new airframe configuration](../airframes/adding_a_new_frame.md)을 제시하는 것입니다. 만약 약간의 수정만을 필요로 한다면 (응용프로그램을 하나더 실행하거나 다른 믹서를 사용하는 것) 스타트업 스크립트 내의 그것들을 위한 훅을 이용하면 됩니다.

> **Caution** 시스템 부팅 파일을 UNIX LINE ENDINGS을 필요로 하는 UNIX 파일입니다. 윈도우에서 수정한다면 적절한 에디터를 사용하세요.

3가지의 메인 훅이 있습니다. microSD 카드의 루트 폴더는 `/fs/microsd` 입니다.

- /fs/microsd/etc/config.txt
- /fs/microsd/etc/extras.txt
- /fs/microsd/etc/mixers/NAME_OF_MIXER

#### config.txt 커스터마이징

`config.txt`는 쉘 변수를 수정하기 위해 사용될 수 있습니다. 이 파일은 기본 시스템이 구성된 후, 부팅되기 전 사이에 로드됩니다.

#### 추가적인 응용프로그램 시작하기

`extras.txt`는 메인 시스템 부팅이후에 추가적인 응용프로그램을 시작하기 위해 사용될 수 있습니다. 보통 이 프로그램들은 컨트롤러나 유사한 선택 커스텀 옵션에 탑재됩니다.

> **Caution** 알 수 없는 명령어를 호출하면 부팅이 실패합니다. 보통 시스템은 부팅 실패 이후에 mavlink 메시지를 스트림처리하지 않습니다. 이 경우에 시스템 콘솔에 출력된 에러 메시지를 확인하세요.

아래의 예제들은 어떻게 커스텀 응용프로그램을 시작하는지 보여줍니다.

- SD 카드 내의 `/etc/extras.txt`에 다음의 내용으로 파일을 만듭니다. ```custom_app start```
- 명령어는 `set +e`과 `set -e` 명령어에 의해 선택적이 될 수 있습니다.
    
        set +e
        optional_app start      # Will not result in boot failure if optional_app is unknown or fails
        set -e
        
        mandatory_app start     # Will abort boot if mandatory_app is unknown or fails
        

#### 커스텀 믹서 시작하기

기본적으로 시스템은 `/etc/mixers`에서 믹서를 로드합니다. 만약 `/fs/microsd/etc/mixers`에 동일한 이름의 믹서가 존재하면 그 믹서를 로드합니다. 이것은 펌웨어를 다시 컴파일할 필요없이 커스텀 믹서를 로드할 수 있게 해줍니다.

##### 예

아래의 예제는 어떻게 커스텀 aux 믹서를 추가하는지 보여줍니다.

- SD 카드 내의 `/etc/mixers/gimbal.aux.mix`에 설정을 채워 파일을 만듭니다.
- 그리고 이걸 사용하기 위해, `/etc/config.txt` 파일을 추가적으로 만들어 다음의 내용을 채워넣습니다. 
        set MIXER_AUX gimbal
        set PWM_AUX_OUT 1234
        set PWM_AUX_DISARMED 1500
        set PWM_AUX_MIN 1000
        set PWM_AUX_MAX 2000
        set PWM_AUX_RATE 50