# 짐벌 제어 설정

이동비행체에 설치된 카메라가 부착된 짐벌(또는 어떤 다른 장비)을 제어하려면, 짐벌을 어떻게 제어 할지 또 어떻게 PX4가 이들을 명령할지 구성해야 합니다. 여기서는 이 설정에 관해 설명합니다.

PX4는 다른 입력과 출력 방법을 갖는 일반적인 마운트/짐벌 제어 드라이버를 갖고 있습니다. 입력은 (예를 들어, 임무 수행이나 측량할 경우) RC나 MAVLink 명령어를 통해 어떻게 짐벌을 제어할지 정의합니다. 출력은 어떻게 이 짐벌이 연결되는지를 정의하는데, MAVLink 명령어를 지원하는 경우가 있고, 다른 PWM을 사용하는 경우가 있습니다. (PWM은 아래에 AUX 출력에서 설명합니다.) 모든 입력 방법은 어떤 방식으로든 출력될 수 있습니다. 두 방식 모두 매개변수를 통해 구성되어야 합니다.

## 매개변수

[매개변수들](../advanced/parameter_reference.md#mount)은 설치 드라이버를 설정하는데 사용됩니다. 가장 중요한 것은 입력(`MNT_MODE_IN`과 출력(`MNT_MODE_OUT`) 모드입니다. 초기 값으로 입력은 활성화되어 있지 않고 드라이버도 작동하지 않습니다. 입력 모드를 선택하면, 재부팅이 되어 설치된 드라이버가 시작됩니다.

입력 모드가 `AUTO`로 설정되어 있으면, 가장 최근의 입력에 근거하여 자동으로 모드가 바뀔 것입니다. MAVLink에서 RC로 전환하기 위해 큰 스틱의 움직임이 필요합니다.

## AUX 출력

출력 모드가 `AUTO`로 설정되어 있으면, 믹서 파일은 출력 핀들에 대한 관계 설정 정의가 필요하고 [설치된 믹서](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/mixers/mount.aux.mix)는 (기체 구성시 정의된 모든 AUX 믹서를 무시하고) 자동으로 선택됩니다.

출력 할당은 다음과 같습니다:

- **AUX1**: Pitch
- **AUX2**: Roll
- **AUX3**: Yaw
- **AUX4**: 셔터/원상복귀

### 믹서 구성 맞춤설정

> **주의** 믹서의 작동 및 믹서 파일의 형식에 대한 설명은 [혼합과 구동기](../concept/mixing.md)를 보세요.

The outputs can be customized by [creating a mixer file](../concept/system_startup.md#starting-a-custom-mixer) on the SD card with name `etc/mixers/mount.aux.mix`.

설치를 위한 기본 믹서 구성은 아래과 같습니다.

    # roll
    M: 1
    O:      10000  10000      0 -10000  10000
    S: 2 0  10000  10000      0 -10000  10000
    
    # pitch
    M: 1
    O:      10000  10000      0 -10000  10000
    S: 2 1  10000  10000      0 -10000  10000
    
    # yaw
    M: 1
    O:      10000  10000      0 -10000  10000
    S: 2 2  10000  10000      0 -10000  10000
    
    

## SITL

Typhoon H480 모델은 미리 설정모의된 짐벌과 함께 제공됩니다. 실행하려면 아래 명령어를 사용하세요:

    make posix gazebo_typhoon_h480
    

다른 모델이나 시뮬레이터에 설치된 드라이버를 시험하려면, 설치된 드라이버가 `vmount start`를 사용해 작동하는지를 확인하고 난 후, 매개 변수들을 구성하십시오.

## 시험하기

이 드라이버는 간단한 시험 명령어를 제공하는데 먼저 `vmount stop`으로 정지시킵니다. 아래는 SITL에서의 시험 방법에 대한 설명이지만, 실제 장비에서도 이 명령어들은 작동합니다.

매개변수가 변경될 필요는 없습니다. 아래 명령어로 시뮬레이션을 시작합니다.

    make posix gazebo_typhoon_h480
    

Armed되어 있는지 확인하세요. 예를 들면, `commander takeoff`를 입력하고 아래 명령어를 사용합니다.

    vmount test yaw 30
    

짐벌이 제어됩니다. 주의할 것은 모의된 짐벌은 자동으로 안정화되므로 Mavlink 명령어들을 통해 `stabilize` 플래그를 false로 설정하세요.

![Gazebo 짐벌 모의](../../assets/gazebo/gimbal-simulation.png)