# 짐벌 제어 설정

비행체에 카메라(또는 다른 물건)를 달아 장착하는 짐벌을 제어하려면, 어떻게 제어할 지, PX4가 어떻게 명령을 내릴지 설정해야합니다. 여기서는 이 설정 방법을 설명합니다.

PX4에는 제각기 다른 입출력 수단에 대한 일반 마운트/짐벌 컨트롤 드라이버가 있습니다. 입력부에서는 리모콘 또는 MAVLink 명령을 통한 짐벌 제어 방식을 정의합니다(예를 들면 missions 또는 survey를 통해). 출력부에서는 짐벌 연결 방식을 정의합니다 일부 짐벌은 MAVLink 명령을 지원하나 다른 제품은 PWM 방식을 활용합니다(아래 내용에서 AUX 출력으로 설명). 어떤 출력이든 제어할 수 있도록 입력 수단을 선택할 수 있습니다. 두 방식 모두 매개변수로 구성해야 합니다.

## 매개변수

[These parameters](../advanced/parameter_reference.md#mount) are used to setup the mount driver. The most important ones are the input (`MNT_MODE_IN`) and the output (`MNT_MODE_OUT`) mode. 초기 값으로 입력은 활성화되어 있지 않고 드라이버도 작동하지 않습니다. After selecting the input mode, reboot the vehicle so that the mount driver starts.

If the input mode is set to `AUTO`, the mode will automatically be switched based on the latest input. To switch from MAVLink to RC, a large stick motion is required.

## AUX 출력

If the output mode is set to `AUX`, a mixer file is required to define the mapping for the output pins and the [mount mixer](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/mixers/mount.aux.mix) is automatically selected (overriding any aux mixer provided by the airframe configuration).

출력 할당은 다음과 같습니다:

- **AUX1**: Pitch
- **AUX2**: Roll
- **AUX3**: Yaw
- **AUX4**: 셔터/원상복귀

### 믹서 구성 맞춤설정

> **주의** 믹서의 작동 및 믹서 파일의 형식에 대한 설명은 [혼합과 구동기](../concept/mixing.md)를 보세요.

출력은 [믹서 파일 만들기](../concept/system_startup.md#starting-a-custom-mixer)로 원하는 데로 변경이 가능하며, SD 카드의 `etc/mixers/mount.aux.mix`에 있습니다.

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

Typhoon H480 모델은 미리 설정모의된 짐벌과 함께 제공됩니다. To run it, use:

    make px4_sitl gazebo_typhoon_h480
    

To just test the mount driver on other models or simulators, make sure the driver runs, using `vmount start`, then configure its parameters.

## 시험하기

이 드라이버는 간단한 시험 명령어를 제공하는데 먼저 `vmount stop`으로 정지시킵니다. 아래는 SITL에서의 시험 방법에 대한 설명이지만, 실제 장비에서도 이 명령어들은 작동합니다.

매개변수가 변경될 필요는 없습니다. 아래 명령어로 시뮬레이션을 시작합니다.

    make px4_sitl gazebo_typhoon_h480
    

Armed되어 있는지 확인하세요. 예를 들면, `commander takeoff`를 입력하고 아래 명령어를 사용합니다.

    vmount test yaw 30
    

짐벌이 제어됩니다. Note that the simulated gimbal stabilizes itself, so if you send MAVLink commands, set the `stabilize` flags to false.

![Gazebo 짐벌 모의](../../assets/simulation/gazebo/gimbal-simulation.png)