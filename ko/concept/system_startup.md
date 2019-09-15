# 시스템 스타트업

PX4 스타트업은 쉘 스크립트에 의해 제어됩니다. 쉘 스크립트는 NuttX는 [ROMFS/px4fmu_common/init.d](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d) 폴더에 있습니다. 몇몇의 Posix 계열(Linux/MacOS)도 동일합니다. Posix만을 위한 스크립트는 [ROMFS/px4fmu_common/init.d-posix](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d-posix)에 위치합니다.

숫자와 밑줄로 시작하는 모든 파일(예. `10000_airplane`)은 기체 설정들을 담고있습니다. 설정값들은 빌드타임에 하나의 `airframes.xml`으로 보내지고 [QGroundControl](http://qgroundcontrol.com)을 통해 기체 선택 UI에 활용됩니다. 새로운 설정을 다루기 위해서는 [here](../airframes/adding_a_new_frame.md)을 참고하세요.

남아 있는 파일들을 일반적인 스타트업 로직을 위해 사용됩니다. 처음 실행되는 파일은 [init.d/rcS](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d/rcS)로 ( 또는 Posix에서는 [init.d-posix/rcS](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d-posix/rcS)), 다른 모든 스크립트들을 호출합니다.

다음의 섹션은 PX4가 실행되는 운영체제에 따라 구분되어 있습니다.

## Posix (Linux/MacOS)

Posix에서는 시스템 쉘이 쉘 인터프리터로 사용됩니다 (예. /bin/sh는 우분에서 대시로 심볼릭링크 됨) 동작하기 위한 몇가지 조건들이 있습니다.

- PX4 모듈은 시스템에서 개별적으로 실행될 수 있어야합니다. 이것은 심볼릭 링크에 의해 수행됩니다. 각 모듈에 대해 심볼릭 링크 `px-4<module>-> px4`가 `bin` 디렉토리에 생성됩니다. 실행될 때, 바이너리의 경로가 (`argv[0]`) 확인되고, 만약 모듈이라면 (`px4-`로 시작), 메인 px4 인트턴스에게 명령을 보냅니다 (자세한건 아래로). **Tip** `px-4` 접두어는 시스템 명령어와의 충돌을 피하기 위해 사용됩니다 (예. `shutdown`), 그리고 `px-4<TAB>`를 타이핑함으로써 쉬운 탭 자동완성도 지원합니다.
- 쉘은 심볼릭 링크를 찾기위한 위치를 알아야합니다. 따라서 스타트업 스크립트를 실행하기전에 심볼릭 링크된 `bin` 디렉토리가 `PATH` 변수에 추가되어야합니다.
- 쉘은 각 모듈을 새로운 (클라이언트) 프로세스로 실행시킵니다. 메인 PX4 인스턴스 (서버)는 쓰레드로 동작하며, 각 클라이언트 프로세스는 메인 PX4와 통신할 수 있어야합니다. 통신은 [UNIX socker](http://man7.org/linux/man-pages/man7/unix.7.html)을 통해 이뤄집니다. 서버는 소켓으로 수신하고, 클라이언트는 소켓에 연결해 명령어를 보낼 수 있습니다. 그러면 서버는 출력과 리턴 코드를 클라이언트에게 보냅니다.
- 스타트업 스크립트는 `px-4`로 시작하는 모듈이 대신 모듈을 직접적으로 호출합니다. 예. `commander start` 이것은 alias를 통해 수행됩니다. 각 모듈에 대해 `alias<module>=px4-<module>`의 형태로 alias가 `bin/px4-alias.sh`에 생성됩니다.
- `rcS` 스크립트는 메인 PX4 인스턴스테 의해 실행됩니다. 이 스크립트는 다른 어떤 모듈들을 실행시키지 않습니다. `PATH` 변수를 업데이트하고 파라미터로 `rcS`을 실행시킵니다.
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

드라이버 하나의 실패가 부팅의 중단을 이끌지는 않습니다. 이것은 스타트업 스크립트에서 `set +e`를 통해 제어됩니다.

부팅 순서는 [system console](../debug/system_console.md)을 통해 디버깅 할 수있습니다. 출력되는 부팅 로그는 부팅 순서에 대한 자세한 정보를 포함하고 왜 부팅이 중단 돼었는지는 포함해야 합니다.

#### 일반적인 부팅 실패 사례

- 커스텀 응용프로그램: 시스템의 메모리 사용량을 벗어납니다. `free` 명령을 수행해 사용가능한 RAM이 얼마인지 확인합니다.
- 스택 트레이스결과 소프트웨어 실패(fault)

### 시스템 스타트업 바꾸기

대부분의 경우에서 부팅을 커스터마이징하는 것이 더 나은 방법입니다. 만약 부팅은 바꿔야 한다면, miscroSD 카드의 `etc` 폴더에 위치한 `/fs/microsd/etc/rc.txt` 파일을 만들어야 합니다. 만약 그 파일에 아무것도 없다면 auto-start가 수행될 것입니다.

### 시스템 스타트업 커스터마이징하기

시스템 스타트업을 커스터마이징하기 위한 최선의 방법은 [new airframe configuration](../airframes/adding_a_new_frame.md)을 제시하는 것입니다. 만약 약간의 수정만을 필요로 한다면 (응용프로그램을 하나더 실행하거나 다른 믹서를 사용) 스타트업 스크립트 내의 그것들을 위한 훅을 이용하면 됩니다.

> **Caution** 시스템 부팅 파일을 UNIX LINE ENDINGS을 필요로 하는 UNIX 파일입니다. 윈도우에서 수정한다면 적절한 에디터를 사용하세요.

3가지의 메인 훅이 있습니다. microSD 카드의 루트 폴더는 `/fs/microsd` 입니다.

- /fs/microsd/etc/config.txt
- /fs/microsd/etc/extras.txt
- /fs/microsd/etc/mixers/NAME_OF_MIXER

#### Customizing the Configuration (config.txt)

The `config.txt` file can be used to modify shell variables. It is loaded after the main system has been configured and *before* it is booted.

#### Starting additional applications

The `extras.txt` can be used to start additional applications after the main system boot. Typically these would be payload controllers or similar optional custom components.

> **Caution** Calling an unknown command in system boot files may result in boot failure. Typically the system does not stream mavlink messages after boot failure, in this case check the error messages that are printed on the system console.

The following example shows how to start custom applications:

- Create a file on the SD card `etc/extras.txt` with this content: ```custom_app start```
- A command can be made optional by gating it with the `set +e` and `set -e` commands:
    
        set +e
        optional_app start      # Will not result in boot failure if optional_app is unknown or fails
        set -e
        
        mandatory_app start     # Will abort boot if mandatory_app is unknown or fails
        

#### Starting a custom mixer

By default the system loads the mixer from `/etc/mixers`. If a file with the same name exists in `/fs/microsd/etc/mixers` this file will be loaded instead. This allows to customize the mixer file without the need to recompile the Firmware.

##### Example

The following example shows how to add a custom aux mixer:

- Create a file on the SD card, `etc/mixers/gimbal.aux.mix` with your mixer content.
- Then to use it, create an additional file `etc/config.txt` with this content: 
        set MIXER_AUX gimbal
        set PWM_AUX_OUT 1234
        set PWM_AUX_DISARMED 1500
        set PWM_AUX_MIN 1000
        set PWM_AUX_MAX 2000
        set PWM_AUX_RATE 50