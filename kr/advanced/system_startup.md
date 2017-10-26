# System Startup

The PX4 부트는 [ROMFS/px4fmu_common/init.d](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d) 폴더에 있는 쉘 스크립트가 제어합니다.

숫자와 밑줄로 시작하는 모든 파일은 airframe 설정과 관련되어 있습니다.(예 `10000_airplane`) 빌드하는 과정에서 `airframes.xml`로 export되며 airframe 선택하는 UI에서 사용하기 위해 [QGroundControl](http://qgroundcontrol.com)에서 파싱됩니다. 새로운 설정을 추가하는 방법은 [여기](../airframes/adding_a_new_frame.md)를 참고하세요.

나머지 파일들은 일반적인 startup 로직의 일부로 사용되며 처음 실행되는 파일은 [rcS](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d/rcS) 스크립트이며 이 스크립트에서 다른 스크립트를 호출하게 됩니다.

## 시스템 부팅 디버깅하기

소프트웨어 컴포넌트 드라이버 중에 하나라도 실패하면 부팅이 실패하게 됩니다.

> **Tip** 불완전하게 부팅되는 경우 ground control station에서 파라미터가 제대로 설정되지 않게 됩니다. 이는 실행되지 않은 application에서 파라미터를 초기화하지 않았기 때문입니다.

부팅 순서를 디버깅하는 올바른 방법은 [system console](../debug/system_console.md)에 연결하여 보드에 전원이 들어오는 과정을 살펴봐야 합니다. 부팅 로그를 보면 부팅 순서에 대해서 상세한 정보를 얻을 수 있기에 왜 부팅에 문제가 있었는지 힌트를 얻을 수 있습니다.

### 일반적인 부팅 실패 원인

  * 필요한 센서의 구동 실패
  * 커스텀 application의 경우 : 시스템의 RAM 메모리가 부족한 경우. `free` 명령을 통해 남은 RAM 메모리 공간을 확인.
  * 소프트웨어 문제나 assertion의 경우 stack trace로 확인 가능

## 시스템 Startup 바꿔치기

일반적으로 기본 부팅에서 바꾸는 것이 좋은 접근법입니다. 이와 관련해서 아래 문서를 참고하세요. 만약 완전히 부팅을 바꿔치기하고자 한다면, `/fs/microsd/etc/rc.txt` 파일을 생성합니다. 이 파일은 마이크로SD 카드의 `etc` 폴더에 위치하고 있습니다. 이 파일이 존재하는 경우 시스템에서 자동으로 시작되는 것이 없어집니다.

## 시스템 Startup 수정하기

시스템 startup을 수정하는 가장 좋은 방법은 [new airframe configuration](../airframes/adding_a_new_frame.md)을 참고합니다. 몇 가지 수정을 하고 싶다면(1개 이상 application을 시작시킨다던가 다른 mixer를 사용하는 경우) 해당 startup에서 수정해서 사용할 수 있습니다.

> **Caution** 시스템 부트 파일은 UNIX 포맷의 파일로 각 라인의 끝에는 UNIX LINED ENDING이 들어가야 합니다. 만약 윈도우에서 수정하는 경우 이를 지원하는 편집기를 이용하도록 합니다.

가지 주요 수정지점이 있습니다. microSD 카드의 루트 폴더는 `/fs/microsd`로 나타냅니다.

  * /fs/microsd/etc/config.txt
  * /fs/microsd/etc/extras.txt
  * /fs/microsd/etc/mixers/NAME_OF_MIXER

### 설정 변경하기 (config.txt)

`config.txt`파일이 로드는 것은 main 시스템이 구성을 마치고 난 후고 부트가 되기 *전* 으로 쉘 변수를 수정하는 것이 가능합니다.

### 추가 application 구동시키기

`extras.txt`는 main 시스템이 부팅되고 나서 추가로 application을 구동시키는데 사용할 수 있습니다. 일반적으로 여기에 해당되는 것은 payload controller나 유사한 선택가능한 커스텀 컴포넌트들입니다.

> **Caution** 시스템 부트 파일에서 알수 없는 명령을 호출하면 부트 실패가 됩니다. 일반적으로 부트 실패가 되면 mavlink 메시지를 내보내지 않습니다. 이 경우 시스템 콘솔에 출력되는 에러 메시지를 확인합니다.

다음 예제에서는 커스텀 어플리케이션을 구동하는 방법에 대해서 알아봅니다:
  * SD 카드 `etc/extras.txt`에 다음과 같은 내용의 파일을 생성:
    ```
    custom_app start
    ```
  * `set +e` 와 `set -e` 명령으로
  A command can be made optional by gating it with the `set +e` and `set -e` commands:
    ```
    set +e
    optional_app start      # optional_app이 없거나 실패하는 경우 부트 실패로 이어지지 않는다
    set -e

    mandatory_app start     # mandatory_app이 없거나 실패하는 경우 부트가 취소된다
    ```  

### 커스텀 믹서 구동시키기

기본적으로 `/etc/mixers`에서 시스템이 믹서를 로드합니다. `/fs/microsd/etc/mixers`에 동일한 이름의 파일이 있는 경우, 이 파일이 대신 로드될 것입니다. 펌웨어를 다시 컴파일할 필요없이 믹서 파일을 수정하는 것이 가능합니다.
#### 예제
다음 예제는 커스텀 aux 믹서를 추가하는 방법을 보여줍니다 :
  * 믹서 관련 내용이 있는 `etc/mixers/gimbal.aux.mix` SD카드에 파일을 생성합니다.
  * 다음으로 이를 이용하기 위해서 아래 내용을 `etc/config.txt` 파일 추가하여 생성합니다.:
    ```
    set MIXER_AUX gimbal
    set PWM_AUX_OUT 1234
    set PWM_AUX_DISARMED 1500
    set PWM_AUX_MIN 1000
    set PWM_AUX_MAX 2000
    set PWM_AUX_RATE 50
    ```
