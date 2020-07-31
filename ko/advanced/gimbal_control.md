# 짐벌 제어 설정

비행체에 카메라(또는 다른 물건)를 달아 장착하는 짐벌을 제어하려면, 어떻게 제어할 지, PX4가 어떻게 명령을 내릴지 설정해야합니다. 여기서는 이 설정 방법을 설명합니다.

PX4에는 제각기 다른 입출력 수단에 대한 일반 마운트/짐벌 컨트롤 드라이버가 있습니다. 입력부에서는 리모콘 또는 MAVLink 명령을 통한 짐벌 제어 방식을 정의합니다(예를 들면 missions 또는 survey를 통해). 출력부에서는 짐벌 연결 방식을 정의합니다 일부 짐벌은 MAVLink 명령을 지원하나 다른 제품은 PWM 방식을 활용합니다(아래 내용에서 AUX 출력으로 설명). 어떤 출력이든 제어할 수 있도록 입력 수단을 선택할 수 있습니다. 두 방식 모두 매개변수로 구성해야 합니다.

## 매개변수

마운트 드라이버를 설정할 때 [이 매개변수](../advanced/parameter_reference.md#mount)를 활용합니다. 가장 중요한 부분은 입력단(`MNT_MODE_IN`)과 출력단(`MNT_MODE_OUT`)의 상태입니다. 기본적으로, 입력단은 활성 상태가 아니며, 드라이버를 실행하고 있지도 않습니다. 입력 상태를 선택하고 나면, 비행체를 재부팅하여 마운트 드라이버를 시작하십시오.

입력단 상태를 `AUTO`로 지정하면, 최근 입력 수단을 기반으로 자동으로 전환합니다. MAVLink에서 리모콘으로 전환하려면, large stick motion이 필요합니다.

## AUX 출력

출력단 상태를 `AUX`로 설정하면, 대응 출력 핀을 정의하고 [마운트 믹서](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/mixers/mount.aux.mix)를 자동으로 선택하는 mixer 파일이 필요합니다(airframe을 설정하여 어떤 AUX mixer 보다 우선함).

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

이 드라이버는 간단한 시험 명령어를 제공합니다. 먼저 `vmount stop`으로 동작을 멈추어야합니다. 아래는 SITL에서의 시험 방법에 대한 설명이지만, 실제 장비에서도 이 명령어들은 작동합니다.

매개변수가 변경될 필요는 없습니다. 아래 명령어로 시뮬레이션을 시작합니다.

    make px4_sitl gazebo_typhoon_h480
    

armed 상태인지 확인하십시오. 예를 들면, `commander takeoff`를 입력하고 아래 명령어를 사용하여

    vmount test yaw 30
    

짐벌을 제어하십시오. 참고로 모의시험 진행시 짐벌은 스스로 안정화를 찾으므로, MAVLink 명령을 보낼 때, `stabilize` 플래그 값을 false로 설정하십시오.

![Gazebo 짐벌 모의시험](../../assets/simulation/gazebo/gimbal-simulation.png)